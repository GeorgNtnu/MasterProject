classdef XPSD_NET < handle 
% class XPSD_NET is meant for communication with the Newport XPS-D 
% controller from Matlab. For this class to work the controller
% hostname adress must be known and the relevant axes must ne set and
% named as single axis groups in the controller web userinterface.
%
% Example usage:
% % First Instansiate the class object containing the handles to the api:
% h = XPSD_NET(Host);  % Host must be a string contaning Hostname or IP
% % Now you can Initialize and home known axis groups using:
% h.GroupInitHome(Group); % Where Group is a string containing the Group 
%                         % name (Set in the XPS web GUI).
% % When succesfully homed, the following methods can be used to move
% % the axis Groups
% h.MoveRelative(Group,Distance); % Distance is a double. The unit is 
%                                 % set in the ini file on the controller
%                                 % Default = [mm]
% h.MoveAbsolute(Group,Position); % Position is a double. The unit is 
%                                 % set in the ini file on the controller
%                                 % Default = [mm]
% Get the current position (=SetpointPosition - FollowingError) of an axis group: 
% Position = h.GetPosition(Group) % Group is a string containing the
%                                 % Groupname
%
% Copyright 2019, TNO, Geerten Kramer

    properties
        SH;     % storage of Soccet handles (in struct)
        asmInfo
        XPS_Hostname = '134.221.115.31'; %http://134.221.115.31/doc/XPS-Unified-ProgrammersManual.pdf
        InitTimeout = 2; %[s]
    end
    properties(SetAccess=private, Hidden)
        StateMatrix; % Structture for looking up the Group State (DO NOT USE DIRECTLY) use GroupStateLookup!!
    end

    methods
        function obj=XPSD_NET(Host)
            % XPSD_NET Constructor, use to initialize and get a handle to the connection.
            % Syntax:
            % h = XPSD_NET(Hostname)
            % where
            % Hostname is the hostname or IP of the XPS controller
            %
            if(nargin>0&&~isempty(Host))
                obj.XPS_Hostname = Host;
            end
            obj.asmInfo = NET.addAssembly('Newport.XPS.CommandInterface');
            % Create 3 sockets to handle different type of communication
            % types:
            SoccetTypes = {'Cmd','Status','GetPos','Abort'};
            obj.SH = struct();
            for(ii=1:length(SoccetTypes))
                obj.SH.(SoccetTypes{ii}) = CommandInterfaceXPS.XPS();
                % Connect:
                if(obj.SH.(SoccetTypes{ii}).OpenInstrument(obj.XPS_Hostname,5001,1000))
                    error('Error connecting to "%s"',obj.XPS_Hostname);
                end
            end
            
            % fill the state matrix
            obj.FillStateMatrix();

            
%             Err=obj.myxps.SetTimeout(1000,1000);
%             if(Err)
%                 fprintf(2,'Error setting timeout: %i\n',Err);
%             end
               
            % Get Firmware version
            [Err, Version]=obj.SH.Cmd.FirmwareVersionGet;
            if(Err)
                obj.DispError(Err)
            else
                disp(Version);
            end
            
            [Status]=obj.GetControllerStatus(); % Get current status
            disp(Status);
            
        end

        function GroupInit(obj,Group)
            % Method Initializes the Group if not already done.
            if(~obj.CheckGroupState(Group,{'NOTINIT'},'DispFail','Message'))
                fprintf('Skipping!\n');
                return; 
            end
            obj.CheckRun(obj.SH.Cmd.GroupInitialize(Group));
            %pause(1); % Add a 1-second delay here
            [~,~,State]=obj.GetGroupStatus(Group);
            Time=tic;
            while(~strcmp(State,'NOTREF'))
                pause(0.1);
                [~,~,State]=obj.GetGroupStatus(Group);
                if(toc(Time)>obj.InitTimeout)
                    error('Initialisation Timeout reached!');
                end
            end
            
        end
        
        function GroupHome(obj,Group)
            % Home the specified group when initialized.
            if(~obj.CheckGroupState(Group,{'NOTREF'},'DispFail','Message'))
                fprintf('Skipping!\n');
                return; 
            end

            fprintf('Started homing.')
            obj.SH.Cmd.GroupHomeSearch(Group); % always gives error -1???
            
            while(1)
                [Cause,~,State]=obj.GetGroupStatus(Group);    
                if(strcmp(State,'READY'))
                    fprintf('done\n');
                    break;
                elseif(strcmp(State,'HOMING'))
                    fprintf('.');
                else % Not in READY or HOMING
                    error('\nHoming failed!\n State:%s\nStatus:%s\n',State,Cause);
                end
                pause(0.1);
            end

        end
        
        function GroupInitHome(obj,Group)
            % Depending on the state the Group is in, this method will
            % attempt to bring the Group in the READY state.

            % Check for DISABLE state
            if(obj.CheckGroupState(Group,{'DISABLE'},'DispFail','None'))
                % Enable the Group:
                fprintf('Group is DISABLE, enabling Group %s...',Group);
                obj.Enable(Group);
                disp('done.');
            end
            % Check for Initialisation
            if(obj.CheckGroupState(Group,{'NOTINIT'},'DispFail','None'))
                % Initialize the Group:
                fprintf('Group is not initialized, init Group %s...',Group);
                obj.GroupInit(Group); % will end in NOTREF if succesful
                disp('done.');
            end
            
            % Check for Reference
            if(obj.CheckGroupState(Group,{'NOTREF'},'DispFail','None'))
                % Home the specified axis Group
                fprintf('Group is not homed, home Group %s...\n',Group);
                obj.GroupHome(Group); % will end in READY if succesfull
            end 
            obj.CheckGroupState(Group,{'READY'},'DispPass','State');
        end
                  
        function CheckRun(obj,Err)
            % Helper fun to get error status
            if(Err)
                [Err2, ErrStr] = obj.SH.Status.ErrorStringGet(Err);
                if(Err2)
                    error('Error getting error: %i, original error was: %i\n',Err2,Err);
                else
                    error('%s\nHaving code: %i\n',ErrStr,Err);
                end
            end
        end
        
        function [StatusString,Status] = GetControllerStatus(obj)
            % Get the status of the controller and flush it.
            [Err,Status] = obj.SH.Status.ControllerStatusGet();
            obj.CheckRun(Err);
            [Err,StatusString] = obj.SH.Status.ControllerStatusStringGet(Status);
            obj.CheckRun(Err);
        end            

        function [StatusString,Status,State] = GetGroupStatus(obj,Group)
            % Check the Status and derive the state of the given axis Group
            % [StatusString,Status,State] = obj.GetGroupStatus(Group);
            [Err,Status] = obj.SH.Status.GroupStatusGet(Group);
            obj.CheckRun(Err);
            % Lookup the State from the description (status string)
            [State,StatusString] = obj.GroupStateLookup(Status);
        end            

        function [State,EntryCause] = GroupStateLookup(obj,Status)
            % Looksup the GroupState from the StateMatrix
            % [State,EntryCause] = obj.GroupStateLookup(Status);
            % Status is the StatusCode received from the controller when
            % quireing GroupStatus.
            % State will be the real state of the Group (independent of the
            % path that lead to the state.
            % EntryCause will also give some info on the path entering the
            % state. 
            if(Status==0)
                State=obj.StateMatrix(107).State;
                EntryCause=obj.StateMatrix(107).EntryCause;
            elseif(Status>=1&&Status<=106)
                State=obj.StateMatrix(Status).State;
                EntryCause=obj.StateMatrix(Status).EntryCause;
            else
                State='UNKNOWN_STATE';
                EntryCause=sprintf('No State found in statetable with StatusCode:%i!',Status);
            end
        end
          
        function [Status] = GetGroupMotionStatus(obj,Group)
            % Check if an axis Group is moving or not.
            [Err,TmpStat] = obj.SH.Status.GroupMotionStatusGet(Group,1);
            obj.CheckRun(Err);
            Status = TmpStat(1);
        end
        
        function MoveRelative(obj,Group,Displacement)
            % Do move the specified axis RELATIVE to the current position. 
            
            if(~obj.CheckGroupState(Group,{'READY','MOVING'},'DispFail','Error')),return; end
            
            obj.SH.Cmd.GroupMoveRelative(Group,Displacement,1);
            % always returns -1 due to motion time-out
%             pause(0.5);
            fprintf('Moving..');
            while(obj.GetGroupMotionStatus(Group))
                fprintf('.');
                pause(.5);
            end
            pause(1);
            [StatusString,Status] = GetGroupStatus(obj,Group);
            if(Status~=12)
                fprintf(2,'Move not finished as expected, message:\n%s\nCode: %i\n',StatusString,Status)
            else
                fprintf('done.\n',Group);
            end
            
        end
        
        function MoveAbsolute(obj,Group,ToPosition)
            % Do move the specified axis to a specified ABSOLUTE position. 
            if(~obj.CheckGroupState(Group,{'READY','MOVING'},'DispFail','Error')),return; end

            % Get velocity setting:
%             Vel=obj.GetMoveParameters(Group);
%             % Get Current axis pos:
%             Pos=obj.GetPosition(Group);
            % Start the motion:
            (obj.SH.Cmd.GroupMoveAbsolute(Group,ToPosition,1)); % CheckRun geeft nog vaak Busy socket meldingen
            %fprintf('Moving..');
%             tb=timebar([],'ManTimeDown',abs(Pos-ToPosition)/Vel...
%                 ,sprintf('Pos=%.3f, Target=%.3f',Pos,ToPosition));
%             while(obj.GetGroupMotionStatus(Group))
%                 % Get current position
%                 Pos=obj.GetPosition(Group);
%                 % Update title string
%                 Title=sprintf('Pos=%.3f, Target=%.3f',Pos,ToPosition);
%                 if(timebar(tb,[],[],Title))
%                     fprintf('Cancel pressed, abording motion!\n');
%                     obj.AbortMotion(Group);
%                     break;
%                 end
%                 pause(.2);
%             end
%             tb.delete();
%             obj.CheckGroupState(Group,{'READY'},'DispPass','State');
%             Pos=obj.GetPosition(Group);
%             fprintf('Current position of group "%s" is %.3f\n',Group,Pos);
         end
        
        function [Vel,Acc,MinJerk,MaxJerk]=GetMoveParameters(obj,Group)
            % Get Positioner Profiler parameters
            %   [Velocity,Acceleration,MinJerkTime,MaxJerkTime]=obj.GetMoveParameters(Group)
            % Where:
            %   Group       = XPS axis group
            %   Velocity    = max Velocity the profile will reach
            %   Acceleration= max Acceleration the profile will reach
            %   MinJerkTime = 
            %   MaxJerkTime = 
            [Err,Vel,Acc,MinJerk,MaxJerk]=obj.SH.GetPos.PositionerSGammaParametersGet([Group '.Pos']);
            obj.CheckRun(Err);
        end
        
        function SetMoveParameters(obj,Group,Vel,Acc,MinJerk,MaxJerk)
            % Set Positioner Profiler parameters
            %   obj.SetMoveParameters(Group,Velocity,Acceleration,MinJerkTime,MaxJerkTime)
            % Where:
            %   Group       = XPS axis group
            %   Velocity    = max Velocity the profile will reach
            %   Acceleration= max Acceleration the profile will reach
            %   MinJerkTime = 
            %   MaxJerkTime = 
            if(~obj.CheckGroupState(Group,{'READY','DISABLE'})),return; end
            
            obj.CheckRun(obj.SH.Cmd.PositionerSGammaParametersSet([Group '.Pos'],Vel,Acc,MinJerk,MaxJerk));
            %[AdjDispl]=PositionerSGammaExactVelocityAjustedDisplacementGet(Positioner,Displacement)
            
        end
        
        function SetSpeed(obj,Group,Speed)
            % Set the speed of the specified group [unit/s]
            % Syntax:
            %   obj.SetSpeed(Group,Speed)
            % Where:
            %   Group    = XPS axis group
            %   Speed    = max Velocity the profile will reach
            [Vel,Acc,MinJerk,MaxJerk]=obj.GetMoveParameters(Group);
            if(Vel~=Speed)
                fprintf('Setting group "%s" Velocity from current value %.2e to %.2e...',Group,Vel,Speed);
                obj.SetMoveParameters(Group,Speed,Acc,MinJerk,MaxJerk);
                fprintf('done.\n');
            else
                fprintf('Velocity of group "%s" already is set to %.2e.\n',Group,Vel);
            end
        end
        
        function Position = GetPosition(obj,Group)
            % Get the current position of the specified axis Group
            %   Position = obj.GetPosition(Group)
            % Where:
            %   Group   = XPS axis group
            
            [Err,TmpPos] = obj.SH.GetPos.GroupPositionCurrentGet(Group,1);
            obj.CheckRun(Err);
            Position = TmpPos(1);
        end
        
        function AbortMotion(obj,Group)
            % Abort motion and go back to ready state
            % obj.AbortMotion(Group)
            % Where:
            %   Group   = XPS axis group
            
            if(~obj.CheckGroupState(Group,{'JOGGING','MOVING'}...
                    ,'DispFail','Message')),return; end
            obj.CheckRun(obj.SH.Abort.GroupMoveAbort(Group));
        end

        
        function Disable(obj,Group)
            % When in READY state, this function sets the specified group 
            % in DISABLE state (Loop is broken but homing is kept).
            % Syntax:
            %   obj.Disable(Group)
            % Where:
            %   Group   = XPS axis group
            if(~obj.CheckGroupState(Group,{'READY'}...
                    ,'DispFail','Warning')),return; end
            obj.CheckRun(obj.SH.Cmd.GroupMotionDisable(Group));
        end
        
        function Enable(obj,Group)
            % When in DISABLE state, this function sets the specified group 
            % in READY state (closes the loop again).
            % Syntax:
            %   obj.Enable(Group)
            % Where:
            %   Group   = XPS axis group
            if(~obj.CheckGroupState(Group,{'DISABLE'}...
                    ,'DispFail','Warning')),return; end
            obj.CheckRun(obj.SH.Cmd.GroupMotionEnable(Group));
            
        end
        
        function OK = CheckGroupState(obj,Group,State,varargin)
            % Checks if a Group is in one of the specified states
            % OK = obj.CheckGroupState(Group,State)
            % OK = obj.CheckGroupState(Group,State,Name1,Value1...)
            % Where:
            %   Group   = XPS groupname
            %   State   = String or cell of strings specifying one or
            %             multiple states. The function check if the group
            %             is in one of this states and returns false
            %             otherwise.
            %   Name,Value pairs can specify:
            %      DispFail     = Does the function display something when
            %                     the test fails? Can be:
            %                     'None' = Dispays nothing, just returns.
            %                     'Message' = Displays a message 
            %                     'Warning' = Issueas a warning (Default)
            %                     'Error' = Issues an Error
            %      DispPass     = Does the function display anything when
            %                     the test passes? Can Be:
            %                     'None' (default)
            %                     'Message'= Displays a message.
            % Copyright 2019, TNO, Geerten Kramer
            if(nargin<3)
                error('Not enough input arguments...')
            elseif(nargin==3)
                Params=struct();
            else
                if(mod(length(varargin),2)) % odd number of args
                    error('Specify even number of arguments (name value pairs, should be pairs)!');
                end
                Params=struct(varargin{:});
            end
            if(~isfield(Params,'DispFail')), Params.DispFail = 'Warning';end
            if(~isfield(Params,'DispPass')), Params.DispPass = 'None';end
            if(~iscell(State))
                State={State};
            end
            [Cause,~,CurState]=obj.GetGroupStatus(Group);
            if(~any(strcmpi(CurState,State)))
                OK = false;
                switch lower(Params.DispFail)
                    case 'none'
                    case 'message'
                        fprintf('Group state is not as expected!\nCurrent State of group "%s": %s\nDescription: %s\n',Group,CurState,Cause);
                    case 'warning'
                        warning('Group state is not as expected!\nCurrent State of group "%s": %s\nDescription: %s\n',Group,CurState,Cause);
                    case 'error'
                        error('Group state is not as expected!\nCurrent State of group "%s": %s\nDescription: %s\n',Group,CurState,Cause);
                    otherwise
                        error('Unsupported DispFail parameter "%s"!',Params.DispFail);
                end
                
            else
                 OK = true; 
                switch lower(Params.DispPass)
                    case 'none'
                    case 'message'
                        fprintf('Current State of group "%s": %s\nDescription: %s\n',Group,CurState,Cause);
                    case 'state'
                        fprintf('Current State of group "%s": %s\n',Group,CurState);
                    otherwise
                        error('Unsupported DispPass parameter "%s"!',Params.DispPass);
                end
                 
            end
        end
        
        
        function Jog(obj,group,dur,vel,acc)
             return;
            if(~obj.CheckGroupState(Group,'JOGGING','DispFail','None'))
                if(~obj.CheckGroupState(Group,'READY','DispFail','Error')),return; end
                % In correct state to enter JOG mode, set it:
                fprintf('In ready state, trying to enter JOG-mode...')
                obj.CheckRun(obj.SH.Cmd.GroupJogModeEnable(group));
                fprintf('done\n')
            end
            if(~obj.CheckGroupState(Group,'JOGGING','DispFail','Error')), return; end
                
            obj.CheckRun(obj.SH.Cmd.GroupJogParametersSet(group,1,vel,acc));
            
            [StatusString,Status] = GetGroupStatus(obj,group);
            if(Status~=47)
                fprintf(2,'Jog not finished as expected, message:\n%s\nCode: %i\n',StatusString,Status)
            else
                fprintf('Jogging with velocity %d mm/s and acceleration %d mm/s^2\n',vel,acc)
            end
            
%             while 1
%                 [cvel, cacc] = obj.myxps.GroupJogCurrentGet(group,1)
%             end
            
            pause(dur)
           
            %obj.myxps.GroupJogParametersSet(group,1,0,acc)
            %obj.myxps.GroupJogModeEnable(group)
            
            obj.AbortMotion(Group);
            [StatusString,Status] = GetGroupStatus(obj,group);
        end
        

        function delete(obj)
            % Should implement the stopping of all axis groups
            % and disabling the axis
            % Nicely close the Instrument connection
            fprintf('XPSD_NET delete function called. Attempting to close Instrumnet...')
            fn=fieldnames(obj.SH);
            for(ii=1:length(fn))
                if(obj.SH.(fn{ii}).CloseInstrument)
                    fprintf(2,'\nError closing Socket "%s"!!\n',fn{ii});
                else
                    disp(' Done.');
                end
            end
            
        end

        function FillStateMatrix(obj)
            % Fills the state matrix (watch out index 0 is mapped to 107!!
            % because of matlab's 1 based indexing.
            obj.StateMatrix(107).EntryCause = 'NOTINIT state';
            obj.StateMatrix(107).State = 'NOTINIT';
            obj.StateMatrix(1).EntryCause = 'NOTINIT state due to an emergency brake: see positioner status';
            obj.StateMatrix(1).State = 'NOTINIT';
            obj.StateMatrix(2).EntryCause = 'NOTINIT state due to an emergency stop: see positioner status';
            obj.StateMatrix(2).State = 'NOTINIT';
            obj.StateMatrix(3).EntryCause = 'NOTINIT state due to a following error during homing';
            obj.StateMatrix(3).State = 'NOTINIT';
            obj.StateMatrix(4).EntryCause = 'NOTINIT state due to a following error';
            obj.StateMatrix(4).State = 'NOTINIT';
            obj.StateMatrix(5).EntryCause = 'NOTINIT state due to an homing timeout';
            obj.StateMatrix(5).State = 'NOTINIT';
            obj.StateMatrix(6).EntryCause = 'NOTINIT state due to a motion done timeout during homing';
            obj.StateMatrix(6).State = 'NOTINIT';
            obj.StateMatrix(7).EntryCause = 'NOTINIT state due to a KillAll command';
            obj.StateMatrix(7).State = 'NOTINIT';
            obj.StateMatrix(8).EntryCause = 'NOTINIT state due to an end of run after homing';
            obj.StateMatrix(8).State = 'NOTINIT';
            obj.StateMatrix(9).EntryCause = 'NOTINIT state due to an encoder calibration error';
            obj.StateMatrix(9).State = 'NOTINIT';
            obj.StateMatrix(10).EntryCause = 'Ready state due to an AbortMove command';
            obj.StateMatrix(10).State = 'READY';
            obj.StateMatrix(11).EntryCause = 'Ready state from homing';
            obj.StateMatrix(11).State = 'READY';
            obj.StateMatrix(12).EntryCause = 'Ready state from motion';
            obj.StateMatrix(12).State = 'READY';
            obj.StateMatrix(13).EntryCause = 'Ready State due to a MotionEnable command';
            obj.StateMatrix(13).State = 'READY';
            obj.StateMatrix(14).EntryCause = 'Ready state from slave';
            obj.StateMatrix(14).State = 'READY';
            obj.StateMatrix(15).EntryCause = 'Ready state from jogging';
            obj.StateMatrix(15).State = 'READY';
            obj.StateMatrix(16).EntryCause = 'Ready state from analog tracking';
            obj.StateMatrix(16).State = 'READY';
            obj.StateMatrix(17).EntryCause = 'Ready state from trajectory';
            obj.StateMatrix(17).State = 'READY';
            obj.StateMatrix(18).EntryCause = 'Ready state from spinning';
            obj.StateMatrix(18).State = 'READY';
            obj.StateMatrix(19).EntryCause = 'Ready state due to a group interlock error during motion';
            obj.StateMatrix(19).State = 'READY';
            obj.StateMatrix(20).EntryCause = 'Disable state';
            obj.StateMatrix(20).State = 'DISABLE';
            obj.StateMatrix(21).EntryCause = 'Disabled state due to a following error on ready state';
            obj.StateMatrix(21).State = 'DISABLE';
            obj.StateMatrix(22).EntryCause = 'Disabled state due to a following error during motion';
            obj.StateMatrix(22).State = 'DISABLE';
            obj.StateMatrix(23).EntryCause = 'Disabled state due to a motion done timeout during moving';
            obj.StateMatrix(23).State = 'DISABLE';
            obj.StateMatrix(24).EntryCause = 'Disabled state due to a following error on slave state';
            obj.StateMatrix(24).State = 'DISABLE';
            obj.StateMatrix(25).EntryCause = 'Disabled state due to a following error on jogging state';
            obj.StateMatrix(25).State = 'DISABLE';
            obj.StateMatrix(26).EntryCause = 'Disabled state due to a following error during trajectory';
            obj.StateMatrix(26).State = 'DISABLE';
            obj.StateMatrix(27).EntryCause = 'Disabled state due to a motion done timeout during trajectory';
            obj.StateMatrix(27).State = 'DISABLE';
            obj.StateMatrix(28).EntryCause = 'Disabled state due to a following error during analog tracking';
            obj.StateMatrix(28).State = 'DISABLE';
            obj.StateMatrix(29).EntryCause = 'Disabled state due to a slave error during motion';
            obj.StateMatrix(29).State = 'DISABLE';
            obj.StateMatrix(30).EntryCause = 'Disabled state due to a slave error on slave state';
            obj.StateMatrix(30).State = 'DISABLE';
            obj.StateMatrix(31).EntryCause = 'Disabled state due to a slave error on jogging state';
            obj.StateMatrix(31).State = 'DISABLE';
            obj.StateMatrix(32).EntryCause = 'Disabled state due to a slave error during trajectory';
            obj.StateMatrix(32).State = 'DISABLE';
            obj.StateMatrix(33).EntryCause = 'Disabled state due to a slave error during analog tracking';
            obj.StateMatrix(33).State = 'DISABLE';
            obj.StateMatrix(34).EntryCause = 'Disabled state due to a slave error on ready state';
            obj.StateMatrix(34).State = 'DISABLE';
            obj.StateMatrix(35).EntryCause = 'Disabled state due to a following error on spinning state';
            obj.StateMatrix(35).State = 'DISABLE';
            obj.StateMatrix(36).EntryCause = 'Disabled state due to a slave error on spinning state';
            obj.StateMatrix(36).State = 'DISABLE';
            obj.StateMatrix(37).EntryCause = 'Disabled state due to a following error on auto-tuning';
            obj.StateMatrix(37).State = 'DISABLE';
            obj.StateMatrix(38).EntryCause = 'Disabled state due to a slave error on auto-tuning';
            obj.StateMatrix(38).State = 'DISABLE';
            obj.StateMatrix(39).EntryCause = 'Disable state due to an emergency stop on auto-tuning state';
            obj.StateMatrix(39).State = 'DISABLE';
            obj.StateMatrix(40).EntryCause = 'Emergency braking';
            obj.StateMatrix(40).State = 'EMERGENCY_BRAKING';
            obj.StateMatrix(41).EntryCause = 'Motor initialization state';
            obj.StateMatrix(41).State = 'MOTOR_INIT';
            obj.StateMatrix(42).EntryCause = 'Not referenced state';
            obj.StateMatrix(42).State = 'NOTREF';
            obj.StateMatrix(43).EntryCause = 'Homing state';
            obj.StateMatrix(43).State = 'HOMING';
            obj.StateMatrix(44).EntryCause = 'Moving state';
            obj.StateMatrix(44).State = 'MOVING';
            obj.StateMatrix(45).EntryCause = 'Trajectory state';
            obj.StateMatrix(45).State = 'TRAJECTORY';
            obj.StateMatrix(46).EntryCause = 'Slave state due to a SlaveEnable command';
            obj.StateMatrix(46).State = 'SLAVE';
            obj.StateMatrix(47).EntryCause = 'Jogging state due to a JogEnable command';
            obj.StateMatrix(47).State = 'JOGGING';
            obj.StateMatrix(48).EntryCause = 'Analog tracking state due to a TrackingEnable command';
            obj.StateMatrix(48).State = 'ANALOG_TRACKING';
            obj.StateMatrix(49).EntryCause = 'Analog interpolated encoder calibrating state';
            obj.StateMatrix(49).State = 'ENCODER_CALIB';
            obj.StateMatrix(50).EntryCause = 'NOTINIT state due to a mechanical zero inconsistency during homing';
            obj.StateMatrix(50).State = 'NOTINIT';
            obj.StateMatrix(51).EntryCause = 'Spinning state due to a SpinParametersSet command';
            obj.StateMatrix(51).State = 'SPINNING';
            obj.StateMatrix(52).EntryCause = 'NOTINIT state due to a clamping timeout';
            obj.StateMatrix(52).State = 'NOTINIT';
            obj.StateMatrix(55).EntryCause = 'Clamped';
            obj.StateMatrix(55).State = 'CLAMPED';
            obj.StateMatrix(56).EntryCause = 'Ready state from clamped';
            obj.StateMatrix(56).State = 'READY';
            obj.StateMatrix(58).EntryCause = 'Disabled state due to a following error during clamped';
            obj.StateMatrix(58).State = 'DISABLE';
            obj.StateMatrix(59).EntryCause = 'Disabled state due to a motion done timeout during clamped';
            obj.StateMatrix(59).State = 'DISABLE';
            obj.StateMatrix(60).EntryCause = 'NOTINIT state due to a group interlock error on not reference state';
            obj.StateMatrix(60).State = 'NOTINIT';
            obj.StateMatrix(61).EntryCause = 'NOTINIT state due to a group interlock error during homing';
            obj.StateMatrix(61).State = 'NOTINIT';
            obj.StateMatrix(63).EntryCause = 'NOTINIT state due to a motor initialization error';
            obj.StateMatrix(63).State = 'NOTINIT';
            obj.StateMatrix(64).EntryCause = 'Referencing state';
            obj.StateMatrix(64).State = 'REFERENCING';
            obj.StateMatrix(65).EntryCause = 'Clamping initialization';
            obj.StateMatrix(65).State = 'CLAMP_INIT';
            obj.StateMatrix(66).EntryCause = 'NOTINIT state due to a perpendicularity error homing';
            obj.StateMatrix(66).State = 'NOTINIT';
            obj.StateMatrix(67).EntryCause = 'NOTINIT state due to a master/slave error during homing';
            obj.StateMatrix(67).State = 'NOTINIT';
            obj.StateMatrix(68).EntryCause = 'Auto-tuning state';
            obj.StateMatrix(68).State = 'AUTOTUNING';
            obj.StateMatrix(69).EntryCause = 'Scaling calibration state';
            obj.StateMatrix(69).State = 'SCALING_CALIB';
            obj.StateMatrix(70).EntryCause = 'Ready state from auto-tuning';
            obj.StateMatrix(70).State = 'READY';
            obj.StateMatrix(71).EntryCause = 'NOTINIT state from scaling calibration';
            obj.StateMatrix(71).State = 'NOTINIT';
            obj.StateMatrix(72).EntryCause = 'NOTINIT state due to a scaling calibration error';
            obj.StateMatrix(72).State = 'NOTINIT';
            obj.StateMatrix(73).EntryCause = 'Excitation signal generation state';
            obj.StateMatrix(73).State = 'EXITATION_SIGNAL';
            obj.StateMatrix(74).EntryCause = 'Disable state due to a following error on excitation signal generation state';
            obj.StateMatrix(74).State = 'DISABLE';
            obj.StateMatrix(75).EntryCause = 'Disable state due to a master/slave error on excitation signal generation state';
            obj.StateMatrix(75).State = 'DISABLE';
            obj.StateMatrix(76).EntryCause = 'Disable state due to an emergency stop on excitation signal generation state';
            obj.StateMatrix(76).State = 'DISABLE';
            obj.StateMatrix(77).EntryCause = 'Ready state from excitation signal generation';
            obj.StateMatrix(77).State = 'READY';
            obj.StateMatrix(78).EntryCause = 'Focus state';
            obj.StateMatrix(78).State = 'FOCUS';
            obj.StateMatrix(79).EntryCause = 'Ready state from focus';
            obj.StateMatrix(79).State = 'READY';
            obj.StateMatrix(80).EntryCause = 'Disable state due to a following error on focus state';
            obj.StateMatrix(80).State = 'DISABLE';
            obj.StateMatrix(81).EntryCause = 'Disable state due to a master/slave error on focus state';
            obj.StateMatrix(81).State = 'DISABLE';
            obj.StateMatrix(82).EntryCause = 'Disable state due to an emergency stop on focus state';
            obj.StateMatrix(82).State = 'DISABLE';
            obj.StateMatrix(83).EntryCause = 'NOTINIT state due to a group interlock error';
            obj.StateMatrix(83).State = 'NOTINIT';
            obj.StateMatrix(84).EntryCause = 'Disable state due to a group interlock error during moving';
            obj.StateMatrix(84).State = 'DISABLE';
            obj.StateMatrix(85).EntryCause = 'Disable state due to a group interlock error during jogging';
            obj.StateMatrix(85).State = 'DISABLE';
            obj.StateMatrix(86).EntryCause = 'Disable state due to a group interlock error on slave state';
            obj.StateMatrix(86).State = 'DISABLE';
            obj.StateMatrix(87).EntryCause = 'Disable state due to a group interlock error during trajectory';
            obj.StateMatrix(87).State = 'DISABLE';
            obj.StateMatrix(88).EntryCause = 'Disable state due to a group interlock error during analog tracking';
            obj.StateMatrix(88).State = 'DISABLE';
            obj.StateMatrix(89).EntryCause = 'Disable state due to a group interlock error during spinning';
            obj.StateMatrix(89).State = 'DISABLE';
            obj.StateMatrix(90).EntryCause = 'Disable state due to a group interlock error on ready state';
            obj.StateMatrix(90).State = 'DISABLE';
            obj.StateMatrix(91).EntryCause = 'Disable state due to a group interlock error on auto-tuning state';
            obj.StateMatrix(91).State = 'DISABLE';
            obj.StateMatrix(92).EntryCause = 'Disable state due to a group interlock error on excitation signal generation state';
            obj.StateMatrix(92).State = 'DISABLE';
            obj.StateMatrix(93).EntryCause = 'Disable state due to a group interlock error on focus state';
            obj.StateMatrix(93).State = 'DISABLE';
            obj.StateMatrix(94).EntryCause = 'Disabled state due to a motion done timeout during jogging';
            obj.StateMatrix(94).State = 'DISABLE';
            obj.StateMatrix(95).EntryCause = 'Disabled state due to a motion done timeout during spinning';
            obj.StateMatrix(95).State = 'DISABLE';
            obj.StateMatrix(96).EntryCause = 'Disabled state due to a motion done timeout during slave mode';
            obj.StateMatrix(96).State = 'DISABLE';
            obj.StateMatrix(97).EntryCause = 'Disabled state due to a ZYGO error during motion';
            obj.StateMatrix(97).State = 'DISABLE';
            obj.StateMatrix(98).EntryCause = 'Disabled state due to a master/slave error during trajectory';
            obj.StateMatrix(98).State = 'DISABLE';
            obj.StateMatrix(99).EntryCause = 'Disable state due to a ZYGO error on jogging state';
            obj.StateMatrix(99).State = 'DISABLE';
            obj.StateMatrix(100).EntryCause = 'Disabled state due to a ZYGO error during analog tracking';
            obj.StateMatrix(100).State = 'DISABLE';
            obj.StateMatrix(101).EntryCause = 'Disable state due to a ZYGO error on auto-tuning state';
            obj.StateMatrix(101).State = 'DISABLE';
            obj.StateMatrix(102).EntryCause = 'Disable state due to a ZYGO error on excitation signal generation state';
            obj.StateMatrix(102).State = 'DISABLE';
            obj.StateMatrix(103).EntryCause = 'Disabled state due to a ZYGO error on ready state';
            obj.StateMatrix(103).State = 'DISABLE';
            obj.StateMatrix(104).EntryCause = 'Driver initialization';
            obj.StateMatrix(104).State = 'DRIVER_INIT';
            obj.StateMatrix(105).EntryCause = 'Jitter initialization';
            obj.StateMatrix(105).State = 'JITTER_INIT';
            obj.StateMatrix(106).EntryCause = 'Not initialized state due to an error with GroupKill or KillAll command';
            obj.StateMatrix(106).State = 'NOTINIT';
            
        end

        
        
    end
    methods(static)
        
    end
    
end
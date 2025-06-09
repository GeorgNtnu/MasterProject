% Load the .mat file
mat_data = load('WaveForm.mat');

% Extract the symbols
symbols = mat_data.TxSymb;

% Open a binary file for writing
fileID = fopen('WaveForm.bin', 'w');

% Interleave real and imaginary parts
symbols_interleaved = [real(symbols(:))'; imag(symbols(:))'];
symbols_interleaved = symbols_interleaved(:);

% Write the interleaved symbols to the binary file
fwrite(fileID, symbols_interleaved, 'float32');

% Close the file
fclose(fileID);


%% Verify

% Open the binary file for reading
fileID = fopen('WaveForm.bin', 'r');

% Read the interleaved symbols from the binary file
symbols_interleaved_read = fread(fileID, 'float32');

% Close the file
fclose(fileID);

% Reconstruct the complex symbols
symbols_read = symbols_interleaved_read(1:2:end) + 1i * symbols_interleaved_read(2:2:end);

% Display the first few symbols to verify
disp('First few complex symbols read from the binary file:');
disp(symbols_read(1:10));


Test = symbols_read(1:278).';
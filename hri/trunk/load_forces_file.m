function [data,sampleRate] = load_forces_file(filename)

fid = fopen(filename);

if ( fid < 0 )
    error([filename ' could not be opened.']);
end

tline = fgetl(fid); % [Force Data]
if ( ~strcmp(tline, '[Force Data]') )
    fclose(fid);
    error([filename ' does not appear to be a .forces file.']);
end

% Begin processing of the header
[numPlates, sampleRate, numSamples] = parse_header(fid);

tline = fgetl(fid); % Column header.. junk

% Read data section
plateformat = '%f%f%f%f%f%f%f';
format = '%*n';
for n = 1:numPlates
    format = [format plateformat];
end

raw_data = textscan(fid, format);
fclose(fid);

data = zeros(numSamples,7,numPlates);
for n = 1:numPlates
    range = (n-1)*7+1:(n-1)*7+7;
    matrix = [raw_data{:,range}];
    data(:,:,n) = matrix;
end



function [np, sr, ns] = parse_header(fid)

% Regular expression template: variable=value
expr = '(?<variable>\w+)=(?<value>\w+)';

tline = fgetl(fid); % Number of force plates
[tokens names] = regexp(tline, expr, 'tokens', 'names');
if ( strcmp(names.variable, 'NumberOfForcePlates') )
    np = sscanf(names.value,'%d');
    %disp(['NumberOfForcePlates = ' num2str(np)]);
else
    warning('NumberOfForcePlates not defined!');
end

tline = fgetl(fid); % Sample rate
[tokens names] = regexp(tline, expr, 'tokens', 'names');
if ( strcmp(names.variable, 'SampleRate') )
    sr = sscanf(names.value,'%d');
    %disp(['SampleRate = ' num2str(sr)]);
else
    warning('SampleRate not defined!');
end

tline = fgetl(fid); % Number of samples
[tokens names] = regexp(tline, expr, 'tokens', 'names');
if ( strcmp(names.variable, 'NumberOfSamples') )
    ns = sscanf(names.value,'%d');
    %disp(['NumberOfSamples = ' num2str(ns)]);
else
    warning('NumberOfSamples not defined!');
end

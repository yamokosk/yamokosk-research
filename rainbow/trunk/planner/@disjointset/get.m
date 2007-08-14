function val = get(ds, propName)
% GET Get asset properties from the specified object
% and return the value
switch propName
case 'Sets'
   val = ds.sets;
otherwise
   error([propName,' Is not a valid asset property'])
end
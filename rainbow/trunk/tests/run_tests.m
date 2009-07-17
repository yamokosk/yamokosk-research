function run_tests()

mfiles = dir('*.m');
test_ran = 0;
test_passed = 0;

for n = 1:length(mfiles)
    
    ind = strfind(mfiles(n).name, 'test_');
    
    if ( ~isempty(ind) )
        
        func_name = mfiles(n).name(1:end-2);
        test_ran = test_ran + 1;
        fprintf(1, '\nTest #%d: %s. ', test_ran, func_name);
        
        try
            eval( func_name );
        catch
            fprintf(1, 'FAILED.\n');
            err = lasterror();
            fprintf(1, '-- On line %d in %s.\n', err.stack(1,1).line, err.stack(1,1).file);
            fprintf(1, '-- Reason: %s\n', err.message); 
            continue;
        end
        test_passed = test_passed + 1;
        fprintf(1, 'PASSED.\n');
    end
    
end

fprintf(1, '\n%d of %d tests passed.\n', test_passed, test_ran);
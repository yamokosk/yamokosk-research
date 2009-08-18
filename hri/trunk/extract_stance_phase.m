function [stance_phase, cutoff, N] = extract_stance_phase(force_data, level_in)

z_data = force_data(:,3);

done = false;
level = level_in;
% while ( ~done )
%     level = input(['Set event detection level [' num2str(level_in) ']: ']);
%     if ( isempty(level) )
%         level = level_in;
%     end
    
    [b,a] = butter(6,(30/(1200/2)));
    z_data_filtered = filtfilt(b,a,z_data);
    
    ind = find(z_data_filtered > level);
    dind = diff(ind);
    gaps = ind( find(dind > 10) );

    heel_strike = min(ind);
    
    % if there are gaps in the data then look for the max index up to the
    % first gap.
    heel_off = -1;
    if ( ~isempty(gaps) )
        heel_off = gaps(1);
    else
        heel_off = max(ind);
    end
    stance_phase = force_data(heel_strike:heel_off,:);
    N = heel_off-heel_strike;

%     x = 1:length(z_data);
%     figure(1);
%     hold on;
%     plot(z_data_filtered);
%     plot(heel_strike,z_data(heel_strike),'*');
%     text(heel_strike,z_data(heel_strike),[num2str(z_data(heel_strike))]);
%     plot(heel_off,z_data(heel_off),'o');
%     text(heel_off,z_data(heel_off),[num2str(z_data(heel_off))]);
% 
%     acceptable = input('Is this acceptable [Y/n]? ', 's');
%     if ( isempty(acceptable) || strcmpi(acceptable,'Y') )
%         done = true;
%     end
%     close(1);
% end

cutoff = level;
function bw = compute_signal_bandwidth(freq, Pxx)

total_power = trapz(freq,Pxx);
cutoff = 0.95 * total_power;

n = 2;
while (1)
    power = trapz(freq(1:n),Pxx(1:n));
    if (power > cutoff)
        break;
    else
        n = n+1;
    end
end

bw = freq(n);
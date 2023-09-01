import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

t = [i for i in range(1, 1001)]
rtt = [27.5, 19.0, 19.5, 18.9, 18.6, 21.3, 18.0, 17.1, 21.8, 18.5, 18.9, 19.5, 19.2, 17.3, 21.3, 19.4, 18.9, 19.8, 22.5,
       22.3, 18.2, 19.7, 24.4, 22.8, 17.7, 19.9, 19.7, 17.8, 18.9, 19.9, 19.6, 19.1, 18.0, 20.6, 23.6, 20.4, 17.1, 19.3,
       17.6, 18.6, 16.7, 17.8, 16.1, 16.9, 21.9, 24.9, 19.1, 18.2, 16.2, 18.8, 24.6, 17.7, 18.0, 19.0, 19.6, 18.0, 19.1,
       21.0, 21.0, 16.0, 18.7, 18.0, 16.1, 18.6, 18.2, 22.5, 17.9, 34.5, 17.1, 17.2, 17.4, 23.7, 17.9, 19.6, 19.3, 22.0,
       19.0, 21.1, 19.2, 18.0, 18.4, 17.5, 19.3, 19.1, 19.3, 19.5, 17.9, 19.5, 17.5, 19.7, 20.2, 18.6, 19.7, 17.7, 19.9,
       18.5, 17.6, 18.8, 20.1, 20.2, 19.1, 19.4, 23.0, 18.2, 27.0, 20.6, 20.6, 17.3, 19.3, 18.4, 19.5, 17.6, 18.9, 17.6,
       20.7, 19.2, 19.1, 21.1, 19.1, 23.3, 18.6, 21.3, 20.7, 19.4, 23.8, 18.9, 21.6, 17.2, 26.2, 19.0, 19.5, 18.3, 16.4,
       17.0, 17.5, 18.8, 19.5, 20.8, 19.0, 18.9, 18.2, 19.9, 13.6, 14.4, 16.6, 16.6, 17.3, 18.6, 15.6, 18.5, 17.0, 18.1,
       15.1, 15.3, 15.9, 14.6, 14.8, 16.6, 16.6, 17.0, 20.1, 18.9, 14.7, 22.3, 15.2, 16.9, 14.6, 14.4, 15.8, 14.2, 15.1,
       14.8, 15.2, 18.9, 16.0, 19.2, 19.1, 15.1, 16.9, 19.8, 20.3, 20.7, 20.7, 17.3, 19.3, 16.9, 16.0, 18.1, 21.2, 18.4,
       20.6, 15.0, 15.8, 15.8, 19.5, 16.4, 17.0, 14.3, 15.7, 20.0, 16.7, 14.7, 15.6, 15.0, 20.3, 19.1, 16.7, 18.9, 17.7,
       19.1, 20.2, 16.9, 16.6, 14.3, 17.4, 17.8, 18.9, 17.4, 28.3, 21.0, 18.8, 20.4, 20.4, 18.4, 21.4, 17.0, 15.7, 20.1,
       16.6, 20.5, 19.9, 16.4, 21.4, 17.2, 20.4, 14.5, 13.1, 21.6, 15.8, 19.5, 21.6, 19.6, 17.1, 16.6, 16.6, 16.3, 17.0,
       14.2, 20.3, 24.7, 37.0, 16.4, 15.1, 14.6, 17.1, 16.1, 15.8, 24.0, 16.1, 20.6, 16.6, 17.0, 16.9, 18.7, 15.1, 15.9,
       23.8, 20.5, 16.6, 16.4, 16.7, 18.3, 17.9, 17.4, 16.6, 15.7, 17.0, 16.3, 17.3, 13.9, 18.6, 14.4, 20.0, 17.3, 19.7,
       18.1, 16.8, 16.1, 15.7, 15.8, 15.9, 18.9, 16.6, 17.7, 13.7, 15.1, 14.8, 16.7, 17.4, 17.0, 15.7, 15.8, 15.9, 17.7,
       15.6, 14.1, 14.7, 16.4, 15.5, 14.3, 14.5, 14.6, 16.6, 16.6, 14.0, 17.6, 20.1, 15.3, 22.9, 17.5, 15.4, 16.7, 16.3,
       15.1, 17.4, 15.7, 16.4, 17.9, 18.7, 14.5, 15.9, 16.6, 16.1, 19.1, 19.0, 18.2, 17.1, 16.1, 18.7, 21.3, 18.8, 17.7,
       20.5, 15.1, 16.0, 16.0, 14.9, 14.3, 14.6, 16.2, 18.4, 15.3, 18.4, 14.5, 15.8, 14.4, 15.2, 14.4, 15.1, 16.9, 19.8,
       17.9, 18.9, 18.0, 20.2, 14.9, 14.8, 16.1, 20.5, 21.6, 16.9, 14.7, 16.6, 13.8, 18.2, 14.1, 14.8, 14.2, 15.8, 20.1,
       18.6, 18.3, 13.5, 17.9, 14.6, 15.3, 14.7, 19.7, 17.8, 18.1, 18.8, 14.2, 16.2, 14.5, 18.4, 13.4, 14.3, 15.3, 15.7,
       15.3, 16.1, 18.9, 13.8, 15.3, 17.4, 14.8, 13.6, 14.4, 20.5, 14.7, 20.9, 17.0, 21.7, 17.0, 15.2, 16.1, 14.5, 18.2,
       15.7, 17.8, 17.5, 14.7, 14.0, 16.6, 15.7, 15.0, 14.3, 17.8, 16.9, 16.8, 14.7, 16.7, 19.2, 18.9, 14.7, 18.7, 20.6,
       14.5, 17.3, 19.4, 18.5, 17.5, 15.7, 15.5, 23.0, 17.6, 16.0, 18.8, 15.3, 17.2, 19.5, 20.4, 14.5, 16.6, 16.5, 16.3,
       16.4, 18.3, 18.9, 15.3, 15.7, 16.2, 17.0, 17.4, 20.3, 14.0, 16.5, 17.6, 20.0, 18.8, 17.1, 17.9, 16.6, 22.4, 15.8,
       14.6, 21.2, 13.5, 15.9, 18.8, 15.9, 15.1, 18.1, 18.0, 16.3, 16.7, 15.6, 19.7, 19.7, 19.2, 14.2, 18.0, 14.3, 17.5,
       16.2, 18.4, 19.6, 17.6, 17.4, 16.9, 18.5, 28.4, 17.4, 16.8, 16.2, 14.8, 17.7, 18.3, 17.2, 14.9, 14.9, 15.1, 17.0,
       15.8, 17.3, 15.3, 18.4, 16.1, 16.9, 17.2, 17.5, 18.3, 22.5, 19.9, 19.8, 16.9, 17.9, 16.7, 17.2, 17.3, 19.7, 16.9,
       15.1, 18.2, 17.5, 18.2, 16.1, 19.2, 17.8, 25.5, 16.1, 19.1, 14.7, 15.6, 14.6, 17.2, 19.2, 20.0, 21.2, 16.9, 19.2,
       13.8, 20.9, 19.3, 18.6, 20.4, 19.1, 23.3, 17.2, 19.1, 17.7, 17.9, 17.2, 16.8, 23.7, 18.0, 17.6, 19.3, 17.4, 19.1,
       18.3, 21.4, 17.6, 16.8, 17.8, 18.6, 19.6, 25.9, 18.8, 19.2, 18.3, 25.9, 17.1, 22.2, 18.2, 19.1, 17.9, 16.9, 17.6,
       19.5, 20.0, 18.4, 17.5, 16.9, 18.7, 18.6, 18.2, 19.3, 18.6, 19.2, 23.0, 18.5, 19.8, 21.5, 18.6, 17.1, 19.8, 19.9,
       17.2, 16.4, 18.8, 17.2, 18.5, 17.8, 16.9, 17.7, 21.4, 17.1, 17.8, 18.1, 16.8, 19.9, 16.9, 18.6, 20.9, 15.0, 14.0,
       17.1, 17.6, 16.8, 14.0, 21.3, 20.9, 17.4, 14.8, 15.2, 16.3, 25.1, 20.4, 18.6, 18.4, 18.5, 21.6, 24.3, 24.3, 17.6,
       22.4, 25.7, 16.4, 19.0, 18.1, 16.9, 22.0, 16.9, 17.3, 20.1, 17.7, 16.6, 21.8, 16.9, 20.9, 19.7, 19.5, 18.5, 19.3,
       18.5, 18.4, 17.7, 17.5, 17.7, 17.9, 18.9, 22.0, 17.5, 18.1, 18.2, 17.9, 17.5, 17.4, 22.2, 17.0, 18.5, 20.7, 19.9,
       19.6, 18.1, 19.9, 18.3, 16.7, 16.4, 18.2, 16.2, 15.9, 18.4, 18.8, 16.3, 16.8, 18.2, 16.4, 16.0, 15.7, 16.6, 18.0,
       20.8, 20.3, 41.9, 20.3, 17.3, 18.0, 20.3, 18.8, 24.1, 17.7, 16.4, 18.5, 20.2, 16.6, 25.4, 16.4, 22.4, 15.7, 18.1,
       16.9, 17.2, 15.9, 19.3, 19.9, 16.9, 18.0, 16.3, 17.1, 17.4, 16.5, 17.5, 17.5, 17.9, 17.0, 16.6, 19.5, 15.5, 27.3,
       21.8, 15.2, 16.9, 15.4, 13.1, 16.2, 19.1, 18.0, 16.6, 17.2, 17.5, 16.8, 15.1, 16.0, 13.4, 19.3, 21.3, 15.8, 13.4,
       13.3, 18.8, 16.1, 18.6, 13.8, 14.2, 12.9, 15.4, 21.3, 20.3, 18.2, 18.7, 17.2, 18.9, 19.1, 19.7, 20.4, 21.1, 18.5,
       17.0, 20.8, 20.8, 18.2, 19.7, 19.5, 20.2, 21.9, 18.3, 20.4, 18.5, 18.5, 18.8, 18.5, 17.1, 14.9, 14.6, 14.5, 15.6,
       15.6, 15.4, 16.3, 15.9, 14.5, 17.5, 13.8, 15.3, 16.5, 17.3, 17.4, 19.0, 16.3, 14.9, 15.4, 16.5, 15.1, 19.6, 13.6,
       13.8, 18.3, 16.4, 17.4, 19.5, 20.4, 14.7, 15.9, 18.4, 15.2, 16.6, 18.8, 18.4, 15.2, 14.4, 17.6, 17.6, 35.1, 14.7,
       19.3, 16.4, 17.1, 33.2, 18.1, 17.9, 22.3, 21.8, 17.6, 14.0, 17.4, 30.8, 14.5, 14.6, 15.2, 15.3, 14.3, 14.1, 16.8,
       14.0, 14.6, 14.8, 17.0, 18.8, 17.9, 20.1, 16.8, 19.1, 16.8, 16.8, 13.4, 17.6, 17.3, 15.8, 14.4, 14.6, 15.8, 15.0,
       14.6, 17.7, 15.6, 17.4, 16.6, 20.7, 15.9, 15.9, 15.4, 15.1, 17.6, 21.7, 16.7, 14.6, 17.7, 19.3, 14.8, 16.7, 14.7,
       15.9, 18.0, 22.1, 22.6, 14.3, 16.4, 16.6, 25.1, 19.0, 16.0, 19.0, 16.1, 15.8, 13.7, 13.2, 18.3, 18.0, 16.3, 18.1,
       13.8, 18.9, 23.1, 23.3, 18.4, 18.5, 16.1, 14.5, 15.7, 17.7, 16.7, 16.1, 17.0, 15.8, 16.4, 20.2, 17.0, 16.2, 15.7,
       16.2, 15.1, 15.1, 14.7, 14.0, 14.8, 15.9, 16.5, 17.6, 19.2, 20.1, 17.1, 18.3, 16.3, 15.6, 18.4, 16.4, 20.2, 18.2,
       17.3, 15.2, 14.4, 16.7, 21.4, 19.8, 17.7, 15.2, 16.2, 14.6, 14.1, 17.4, 15.6, 17.2, 14.2, 16.0, 15.3, 17.2, 16.4,
       15.1, 14.9, 16.5, 14.8, 14.6, 15.0, 16.7, 16.2, 15.8, 15.2, 16.1, 16.8, 14.9, 14.8, 14.8, 14.5, 15.6, 16.1, 15.2,
       16.7, 14.9, 14.9, 13.7, 14.6, 15.1, 18.6, 23.4, 14.9, 16.1, 16.6, 15.0]

average = sum(rtt) / len(rtt)
minimum = min(rtt)
maximum = max(rtt)

mean = np.mean(rtt)
std_dev = np.std(rtt)

# Calculate Modified Thompson Tau critical value for a given significance level
significance_level = 0.05
sample_size = len(rtt)
degrees_of_freedom = sample_size - 1
critical_value = stats.t.ppf(1 - significance_level / (2 * sample_size), degrees_of_freedom)

# Calculate Thompson Tau test statistic for each data point
tau_statistics = [(x - mean) / std_dev for x in rtt]

rtt_clean = [rtt[i] for i, tau in enumerate(tau_statistics) if abs(tau) <= critical_value]
t_clean = [i for i in range(1, len(rtt_clean) + 1)]

ci = 1.96 * np.std(rtt) / np.sqrt(len(t))

print(average)
print(ci)

plt.plot(t_clean, rtt_clean, c='red', lw=1)
#plt.plot(t, rtt, c='blue', lw=1)
# plt.fill_between(t, (rtt-ci), (rtt+ci), color='b', alpha=.1)
plt.axhline(average)
plt.axhline(minimum)
plt.axhline(maximum)
plt.axhline(max(rtt_clean))
plt.axhline(min(rtt_clean))
plt.title('RTT h2-h3 L2Fwd 1c')
plt.ylabel('Latency (ms)')
plt.ylim(10, 45)
plt.xlabel('Time')
plt.xlim(1, 1000)

plt.show()
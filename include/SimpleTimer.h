#ifndef SIMPLETIMER_H
#define SIMPLETIMER_H

#include <sys/time.h>
#define MAX_LOOPRATE 100.0f
class SimpleTimer
{
public:
	SimpleTimer()
	{
			gettimeofday(&m_startTime,NULL);
			gettimeofday(&m_lastTime,NULL);
            counter = 0;
            target_rate = -1.0;
            set_rate = -1.0;
            actual_rate = -1.0;
			time_sincelast = 0.0;
			enabled = false;
			looprate_percerror = 0.0;
			last_error = 0.0;
			cum_error = 0.0;
			dt = 0.0;
			print_data = false;
	}
	void reset()
	{
        counter = 0;
		gettimeofday(&m_startTime,NULL);
		gettimeofday(&m_lastTime,NULL);
	}
	void set_name(std::string v) { name = v; }
	std::string get_name() { return name; }
	double timeElapsed()
	{
		gettimeofday(&m_currentTime,NULL);
		return getTimeInSeconds(m_currentTime)-getTimeInSeconds(m_startTime);
	}
	double get_timedelta() { return dt; }
	bool check_looprate()
	{
		if(counter < 10)
		{
			actual_rate = target_rate;
			return true;
		}
		actual_rate = get_actualrate();
		double error = target_rate - actual_rate;
		cum_error += error;
		double P = 5.0*error;
		double I = 1.0*cum_error;
		double D = -1*(error-last_error);
		double t_set_rate = target_rate + P + I + D;
		double t_set_perc = abs(100.0*(target_rate-t_set_rate)/target_rate);
		if(t_set_perc < 20.0)
		{
			set_rate = t_set_rate;
		}
		last_error = error;
		looprate_percerror = 100.0*(target_rate - actual_rate)/target_rate;
		if(name == "FASTLOOP")
		{
			//printf("%f %f %f %f %f %f %f %f %f %f\n",error,last_error,cum_error,P,I,D,set_rate,actual_rate,target_rate,looprate_percerror);
		}
		if(looprate_percerror > 5.0)
		{
			if(print_data == true)
			{
				//printf("WARN: %s is is Running Slow: %4.2f/%4.2f\n",get_name().c_str(),actual_rate,target_rate);
			}
			return false;
		}
		else
		{
			return true;
		}
	}
	double get_timingerrorperc() { return looprate_percerror; }
	double get_currentTime()
	{
		gettimeofday(&m_currentTime,NULL);
		return getTimeInSeconds(m_currentTime);
	}
	bool run_loop()
	{
		gettimeofday(&m_currentTime,NULL);
		bool should_run = false;
		if(enabled == false)
		{
			return false;
		}
		dt = getTimeInSeconds(m_currentTime)-getTimeInSeconds(m_lastTime);
		if(dt > 1.0/set_rate)
		{
			should_run = true;
			time_sincelast = dt;
		}
		if(should_run == true)
		{
            counter++;
			gettimeofday(&m_lastTime,NULL);
		}
		
		return should_run;
	}
	void set_targetrate(double v)
	{
		if(v > MAX_LOOPRATE)
		{
			printf("WARN: %s has a set rate of %4.2f Hz but the max rate is %4.2f Hz.  Reducing to %4.2f Hz.\n",
					get_name().c_str(),
					v,
					MAX_LOOPRATE,
					MAX_LOOPRATE);
			v = MAX_LOOPRATE;
		}

		target_rate = v;
		set_rate = 1.1*v;
		if(target_rate <= 0.0)
		{
			enabled = false;
		}
		else
		{
			enabled = true;
		}
	}
	double get_rate() { return target_rate; }
	double get_setrate() { return set_rate; }
    double get_actualrate() 
    {
        return (double)(counter)/timeElapsed();
    }
	double get_timesincelastrun() { return time_sincelast; }
	void enable_printing() { print_data = true; }
private:
	double getTimeInSeconds(const struct timeval& time)
	{
		return (double)time.tv_sec + (double)(time.tv_usec)/1000000.0;
	}
	bool print_data;
	double last_error;
	double cum_error;
	double looprate_percerror;
    uint64_t counter;
	std::string name;
	struct timeval m_startTime;
	struct timeval m_currentTime;
	double target_rate;
	double actual_rate;
	double set_rate;
	struct timeval m_lastTime;
	double time_sincelast;
	bool enabled;
	double dt;
};
#endif

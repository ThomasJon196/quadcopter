

float p_term, i_term, d_term;


float calc_pid(float Kpid[], float error, float integrated_error, float last_reading, int time_delta)
{
  p_term = Kpid[0] * error;

  integrated_error = integrated_error + error;
  i_term = Kpid[1] * integrated_error;
  
  d_term = Kpid[2] * (last_reading - error); // /time_delta;
  
  return p_term + i_term + d_term;
}

#include <sparki.h>

#define M_PI 3.14159
#define CYCLE_TIME .01
#define DIST_ERR .005
#define HEAD_ERR .03
#define BEAR_ERR .0001

class Sparki
{
    public:
        float x = 0;
        float y = 0;
        float theta = 0;
        float wheel_radius = .03;
        float _speed = .0275;
        float target_x;
        float target_y;
        float target_theta;
        float axle_length = .0857;
        float d_gain_max= 0.1;
        float d_gain = d_gain_max;
        float b_gain_max= 0.01;
        float b_gain = b_gain_max;
        float h_gain_max = 0.1;
        float h_gain = 0;
        float distance;
        float orig_distance;
        float heading;
        float bearing;
        float l_speed = 0;
        float r_speed = 0;
        int lost = 0;
        
        Sparki(float tar_x, float tar_y, float tar_theta)
        {
            target_x = tar_x;
            target_y = tar_y;
            orig_distance = sqrt((tar_x * tar_x) + (tar_y * tar_y));
            distance = orig_distance;
            if (tar_theta > 2 * M_PI) tar_theta -= 2.*M_PI;
            if (tar_theta < 0) tar_theta += 2.*M_PI;
            heading = tar_theta;
            target_theta = tar_theta;
            if(tar_x == 0 && tar_y > 0)
                bearing = M_PI / 2;
            else if(tar_x == 0 && tar_y < 0)
                bearing = - M_PI / 2;
            else if(tar_x == 0)
              bearing = 0;
            else
                bearing = distance * tan(tar_y / tar_x);
            
        }
        
        float delta_xy()
        {
            return (l_speed + r_speed) * wheel_radius / 2;
        }
        
        float delta_theta()
        {
            return (r_speed - l_speed) * wheel_radius / axle_length;
        }
        
        void update_distance()
        {
            distance = sqrt(pow(x - target_x, 2) + pow(y - target_y, 2));
            if(distance < DIST_ERR)
                d_gain = 0;
            else
                if(orig_distance == 0)
                  d_gain = 0;
                else
                  d_gain = .5;
        }
        
        void update_bearing()
        {
            if(x - target_x == 0)
            {
                if(y - target_y > 0)
                    bearing = M_PI / 2;
                else if(y - target_y < 0)
                    bearing = - M_PI / 2;
                else
                    bearing = 0;
            }
            else
                bearing = atan((y - target_y) / (x - target_x));
                
            if (bearing > 2 * M_PI) bearing -= 2.*M_PI;
            if (bearing < -2 * M_PI) bearing += 2.*M_PI;
            
            if(fabs(bearing) < BEAR_ERR)
                b_gain = 0;
            else
            {
                if(orig_distance == 0)
                  b_gain = 0;
                else
                  b_gain = .05;
            }
        }
        
        void update_heading()
        {   
            heading = target_theta - theta;
            if(fabs(heading) < HEAD_ERR)
              h_gain = 0;
            else
                if(distance == 0)
                  h_gain = h_gain_max;
                else
                  h_gain = heading / (2 * M_PI) * h_gain_max;
        }
        
        void update_wheel_speeds(float* l_speed_pct, float* r_speed_pct)
        {
            l_speed = (2 * distance * d_gain) - (axle_length * ((bearing * b_gain) + (heading * h_gain))) / (2 * wheel_radius);
            r_speed = (2 * distance * d_gain) + (axle_length * ((bearing * b_gain) + (heading * h_gain))) / (2 * wheel_radius);
            
            float _max = max(l_speed, r_speed);
            *l_speed_pct = l_speed / _max;
            *r_speed_pct = r_speed / _max;

            if(distance < DIST_ERR)
            {
                b_gain = 0;
                if(fabs(heading) < HEAD_ERR)
                {
                    l_speed, r_speed = 0;
                    *r_speed_pct, *l_speed_pct = 0;
                }   
                /*
                else
                {   
                    r_speed = _speed;
                    l_speed = - r_speed;
                    *r_speed_pct = 1;
                    *l_speed_pct = -1;
                }
                */
            }
            else if(distance > 1.5 * orig_distance)
            {
                    r_speed = _speed;
                    l_speed = 0;
                    *r_speed_pct = 1;
                    *l_speed_pct = -1;
            }
            else
            {
              r_speed = *r_speed_pct * _speed;
              l_speed = *l_speed_pct * _speed;
            }
        }
        
        void step(float* l_speed_pct, float* r_speed_pct)
        {
            // calculate the distance travelled
            float distance_travelled = delta_xy();
            
            float d_theta = delta_theta();
            theta += d_theta;

            if (theta > 2 * M_PI) theta -= 2.*M_PI;
            if (theta < -2 * M_PI) theta += 2.*M_PI;
            
            x += distance_travelled * cos(theta - (d_theta / 2));
            y += distance_travelled * sin(theta - (d_theta / 2));
            
            update_distance();
            update_bearing();
            update_heading();
            
            update_wheel_speeds(l_speed_pct, r_speed_pct);
        }
};

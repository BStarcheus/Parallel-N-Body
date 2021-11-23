#include <string>

class Body{
    public:
        std::string name;
        float mass;
        float radius;
        float pos_x, pos_y;
        float vel_x, vel_y;
        bool hasCollided; // boolean to keep track of which two bodies have collided
        
        Body(std::string n, float m, float r, float px, float py, float vx, float vy, bool hasCollided){
            name = n;
            mass = m;
            radius = r;
            pos_x = px;
            pos_y = py;
            vel_x = vx;
            vel_y = vy;
            hasCollided = hasCollided;
        }
    
    private:
};
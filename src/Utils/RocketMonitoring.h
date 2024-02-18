#include "gladiator.h"

#define ROCKET_SPEED 0.6
#define TIME_ROCKET_ON_MAP 5.0*ROCKET_SPEED*1000.0

struct pos_2d
{
    float i;
    float j;
};

typedef struct {
    pos_2d start;
    pos_2d end;
} Segment;

float distance(pos_2d p1, pos_2d p2);

unsigned char closest_robot_id_to_pos(pos_2d pos, Gladiator *gladiator, bool include_self);

bool is_out_of_bounds(int i, int j);

double distancePointToSegment(pos_2d point, Segment segment);

double dotProduct(pos_2d p1, pos_2d p2);

double distanceSquared(pos_2d p1, pos_2d p2);

class RocketMonitoring
{
public:
    bool Rockets_on_map[16];
    pos_2d Rockets_on_map_start_pos[16];
    unsigned long Rockets_on_map_start_time[16];
    pos_2d Rockets_on_map_end_pos[16];
    int last_index = 0;
    bool rocket_in_map = false;

    bool danger_squares[12][12] = {false};


    RocketMonitoring();
    void check_for_new_rockets(Gladiator* gladiator);
    void set_new_rocket_on_map(int index, pos_2d pos, Gladiator *gladiator);
    void monitoring_loop(Gladiator *gladiator);
    bool aimed_at_me(Gladiator *gladiator);
    void print_info(Gladiator *gladiator);
};

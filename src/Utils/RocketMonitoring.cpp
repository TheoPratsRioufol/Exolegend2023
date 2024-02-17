#include "RocketMonitoring.h"

float distance(pos_2d p1, pos_2d p2)
{
    return sqrt(pow(p1.i - p2.i, 2) + pow(p1.j - p2.j, 2));
}

unsigned char closest_robot_id_to_pos(pos_2d pos, Gladiator *gladiator, bool include_self)
{
    unsigned char closest_robot_id;
    float min_distance = 1000;
    RobotList robots = gladiator->game->getPlayingRobotsId();
    for (int i = 0; i < 4; i++)
    {
        RobotData robot = gladiator->game->getOtherRobotData(robots.ids[i]);
        if ((include_self || robot.id != gladiator->robot->getData().id) && robot.lifes)
        {
            pos_2d robot_pos = {robot.position.x, robot.position.y};
            float dist = distance(pos, robot_pos);
            if (dist < min_distance)
            {
                min_distance = dist;
                closest_robot_id = robot.id;
            }
        }
    }
    return closest_robot_id;
}

bool is_out_of_bounds(int i, int j)
{
    return (i < 0 || i >= 12 || j < 0 || j >= 12);
}

RocketMonitoring::RocketMonitoring()
{
    for (int i = 0; i < 16; i++)
    {
        Rockets_on_map[i] = false;
    }
}

void RocketMonitoring::check_for_new_rockets(Gladiator *gladiator)
{
    for (int i = 0; i < 12; i++)
    {
        for (int j = 0; j < 12; j++)
        {
            const MazeSquare *square = gladiator->maze->getSquare(i, j);
            // compare with RocketMonitoring::danger_squares
            // if a square is dangerous AND all 8 surrounding squares are within bounds and not dangerous,
            // then a rocket has been launched from this square
            // log square danger
            // gladiator->log("Danger at (%d, %d): %d", i, j, square->danger);
            if (square->danger &&
                ((!is_out_of_bounds(i - 1, j - 1) && !danger_squares[i - 1][j - 1]) || is_out_of_bounds(i - 1, j - 1)) &&
                ((!is_out_of_bounds(i - 1, j) && !danger_squares[i - 1][j]) || is_out_of_bounds(i - 1, j)) &&
                ((!is_out_of_bounds(i - 1, j + 1) && !danger_squares[i - 1][j + 1]) || is_out_of_bounds(i - 1, j + 1)) &&
                ((!is_out_of_bounds(i, j - 1) && !danger_squares[i][j - 1]) || is_out_of_bounds(i, j - 1)) &&
                ((!is_out_of_bounds(i, j + 1) && !danger_squares[i][j + 1]) || is_out_of_bounds(i, j + 1)) &&
                ((!is_out_of_bounds(i + 1, j - 1) && !danger_squares[i + 1][j - 1]) || is_out_of_bounds(i + 1, j - 1)) &&
                ((!is_out_of_bounds(i + 1, j) && !danger_squares[i + 1][j]) || is_out_of_bounds(i + 1, j)) &&
                ((!is_out_of_bounds(i + 1, j + 1) && !danger_squares[i + 1][j + 1]) || is_out_of_bounds(i + 1, j + 1)) &&
                !danger_squares[i][j])
            {

                pos_2d rocket_pos = {((float)i + 0.5) * 3.0 / 12.0, ((float)j + 0.5) * 3.0 / 12.0};
                set_new_rocket_on_map(last_index, rocket_pos, gladiator);
                last_index++;
            }
            // update danger_squares
            danger_squares[i][j] = square->danger;
        }
    }
}

void RocketMonitoring::set_new_rocket_on_map(int index, pos_2d pos, Gladiator *gladiator)
{
    // log here
    // gladiator->log("Rocket launched at (%f, %f)", pos.i, pos.j);
    if (index < 16)
    {
        RocketMonitoring::Rockets_on_map[index] = true;
        RocketMonitoring::Rockets_on_map_start_pos[index] = pos;
        unsigned char closest_robot_id = closest_robot_id_to_pos(pos, gladiator, false);
        RobotData robot = gladiator->game->getOtherRobotData(closest_robot_id);
        RocketMonitoring::Rockets_on_map_start_time[index] = millis();
        // log here
        // gladiator->log("Rockets_on_map_start_time: %d", RocketMonitoring::Rockets_on_map_start_time[index]);
        float angle = robot.position.a;
        RocketMonitoring::Rockets_on_map_end_pos[index] = {pos.i + 5 * cos(angle), pos.j + 5 * sin(angle)};
    }
}

void RocketMonitoring::monitoring_loop(Gladiator *gladiator)
{
    RocketMonitoring::check_for_new_rockets(gladiator);
    RocketMonitoring::rocket_in_map = false;
    for (int i = 0; i < 16; i++)
    {
        if (RocketMonitoring::Rockets_on_map[i])
        {
            if (millis() - RocketMonitoring::Rockets_on_map_start_time[i] > TIME_ROCKET_ON_MAP)
            {
                RocketMonitoring::Rockets_on_map[i] = false;
            }
            RocketMonitoring::rocket_in_map = true;
        }
    }
}
// gladiator->log
void RocketMonitoring::print_info(Gladiator *gladiator)
{
    for (int i = 0; i < 16; i++)
    {
        if (RocketMonitoring::Rockets_on_map[i])
        {
            // gladiator->log("Rocket %d: (%f, %f) -> (%f, %f)", i, RocketMonitoring::Rockets_on_map_start_pos[i].i, RocketMonitoring::Rockets_on_map_start_pos[i].j, RocketMonitoring::Rockets_on_map_end_pos[i].i, RocketMonitoring::Rockets_on_map_end_pos[i].j);
        }
    }
}
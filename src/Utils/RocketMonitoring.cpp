#include "RocketMonitoring.h"
#include "math.h"

float distance(pos_2d p1, pos_2d p2)
{
    return sqrt(pow(p1.i - p2.i, 2) + pow(p1.j - p2.j, 2));
}

unsigned char closest_robot_id_to_pos(pos_2d pos, Gladiator *gladiator, bool include_self)
{
    unsigned char closest_robot_id = -1;
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

                pos_2d rocket_pos = {(0.5f+i) * 3.0f / 12.0f, (0.5f+j) * 3.0f / 12.0f};
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

double distanceSquared(pos_2d p1, pos_2d p2) {
    return (p1.i - p2.i) * (p1.i - p2.i) + (p1.j - p2.j) * (p1.j - p2.j);
}

double dotProduct(pos_2d p1, pos_2d p2) {
    return p1.i * p2.i + p1.j * p2.j;
}

double distancePointToSegment(pos_2d point, Segment segment) {
    double segmentLengthSquared = distanceSquared(segment.start, segment.end);
    if (segmentLengthSquared == 0.0) // Segment is actually a point
        return sqrt(distanceSquared(point, segment.start));

    // Calculate the vector from the segment start to the point
    pos_2d segmentToPoint;
    segmentToPoint.i = point.i - segment.start.i;
    segmentToPoint.j = point.j - segment.start.j;

    // Calculate the vector representing the segment
    pos_2d segmentVector;
    segmentVector.i = segment.end.i - segment.start.i;
    segmentVector.j = segment.end.j - segment.start.j;

    // Calculate the parameter t
    double t = dotProduct(segmentVector, segmentToPoint) / segmentLengthSquared;

    if (t < 0.0) // Closest point is the segment start
        return sqrt(distanceSquared(point, segment.start));
    else if (t > 1.0) // Closest point is the segment end
        return sqrt(distanceSquared(point, segment.end));
    else {
        // Closest point is along the segment
        pos_2d closestPoint;
        closestPoint.i = segment.start.i + t * segmentVector.i;
        closestPoint.j = segment.start.j + t * segmentVector.j;
        return sqrt(distanceSquared(point, closestPoint));
    }
}
RobotData robot;
pos_2d robot_pos;
pos_2d rocket_dir;
pos_2d rocket_now;
pos_2d robot_to_rocket;
Segment rocket_path;
bool RocketMonitoring::aimed_at_me(Gladiator *gladiator){
    // calculate distance between robot and rocket path
    robot = gladiator->robot->getData();
    robot_pos = {robot.position.x, robot.position.y};

    for (int i = 0; i < 16; i++)
    {
        if (RocketMonitoring::Rockets_on_map[i])
        {
            // check if rocket is going toward robot
            rocket_dir = {RocketMonitoring::Rockets_on_map_end_pos[i].i - RocketMonitoring::Rockets_on_map_start_pos[i].i, RocketMonitoring::Rockets_on_map_end_pos[i].j - RocketMonitoring::Rockets_on_map_start_pos[i].j};
            // rocket goes in a straith line at a speed of ROCKET_SPEED m/s
            float angle_rocket_path = atan2(rocket_dir.j, rocket_dir.i);
            rocket_now = {ROCKET_SPEED * cos(angle_rocket_path)+RocketMonitoring::Rockets_on_map_start_pos[i].i, ROCKET_SPEED * sin(angle_rocket_path)+RocketMonitoring::Rockets_on_map_start_pos[i].j};

            // calculatethe scalar product between the rocket direction and the vector robot_pos - rocket_now
            robot_to_rocket = {robot_pos.i - rocket_now.i, robot_pos.j - rocket_now.j};
            double scalar_product = rocket_dir.i * robot_to_rocket.i + rocket_dir.j * robot_to_rocket.j;
            if (scalar_product < 0)
            {
                continue;
            }

            // check if distance to rocket path is small
            rocket_path = {RocketMonitoring::Rockets_on_map_start_pos[i], RocketMonitoring::Rockets_on_map_end_pos[i]};
            double dist = distancePointToSegment(robot_pos, rocket_path);
            if (dist < 2.0*3.0/12.0)
            {
                return true;
            }
        }
    }
    return false;
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
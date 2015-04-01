// UGVNavigation.cpp
#include "UGVNavigation.h"


std::vector<double> obstacleRecognition(double * distances, double * thetas, int readings) {
    std::vector<double> obstacle_indexes;
    double readingX[readings];
    double readingY[readings];
    double distance_x, distance_y, points_distance_sq;
    double safety_edge = 20;
    double safety_gap_sq = 4 * safety_edge * safety_edge;

    readingX[0] = distances[0] * cos(thetas[0]);
    readingY[0] = distances[0] * sin(thetas[0]);

    int first_obstacle_end = -1;
    int i = 1;
    while (first_obstacle_end < 0 && i<readings) {
        readingX[i] = distances[i] * cos(thetas[i]);
        readingY[i] = distances[i] * sin(thetas[i]);
        distance_x = readingX[i] - readingX[i - 1];
        distance_y = readingY[i] - readingY[i - 1];
        points_distance_sq = distance_x * distance_x + distance_y * distance_y;
        if (points_distance_sq > safety_gap_sq) {
            first_obstacle_end = i - 1;
            i++;
            break;
        }
        i++;
    }
    while (i < readings) {
        readingX[i] = distances[i] * cos(thetas[i]);
        readingY[i] = distances[i] * sin(thetas[i]);
        i++;
    }

    int number_of_obstacles = 0;
    int start_index = first_obstacle_end + 1;
    int end_index = first_obstacle_end;
    int obstacle_element = start_index;
    bool end_found = false;
    if(first_obstacle_end<0){first_obstacle_end+=readings;}
    while (end_index!=first_obstacle_end || number_of_obstacles==0) { // one iteration for each new obstacle
        number_of_obstacles++;
        start_index = end_index + 1;
        obstacle_indexes.push_back(start_index);
        std::cout<<start_index<<" ";
        obstacle_element = start_index;// set temporary end equal to beginning
        end_index = start_index;
        end_found = false;
        if(number_of_obstacles>10){break;}
        
        while (!end_found && obstacle_element<=readings){ // one iteration for each element in obstacle
            for (int iterator=first_obstacle_end;iterator>=0;iterator--){ // add 90 degree limit
                distance_x = readingX[iterator] - readingX[obstacle_element];
                distance_y = readingY[iterator] - readingY[obstacle_element];
                points_distance_sq = distance_x * distance_x + distance_y * distance_y;
                
                if(points_distance_sq<safety_gap_sq){
                    end_index = iterator;// set new end = iterator
                    end_found = true;
                    break;
                }
            }
            if (!end_found){
                for(int iterator=readings;iterator>=obstacle_element;iterator--){
                    distance_x = readingX[iterator] - readingX[obstacle_element];
                    distance_y = readingY[iterator] - readingY[obstacle_element];
                    points_distance_sq = distance_x * distance_x + distance_y * distance_y;
                    
                    if(points_distance_sq<safety_gap_sq){
                        end_index = iterator;// set new end = iterator
                        end_found = true;
                        break;
                    }
                }
            }
            if (obstacle_element==end_index){
                end_found = true;
                break;
            }
            obstacle_element++;
        }
        obstacle_element = 0;
        while (!end_found){ // one iteration for each element in obstacle
            for (int iterator=first_obstacle_end;iterator>obstacle_element;iterator--){
                distance_x = readingX[iterator] - readingX[obstacle_element];
                distance_y = readingY[iterator] - readingY[obstacle_element];
                points_distance_sq = distance_x * distance_x + distance_y * distance_y;
                
                if(points_distance_sq<safety_gap_sq){
                    end_index = iterator;// set new end = iterator
                    end_found = true;
                    break;
                }
            }
            // if (!end_found){
            //     for(int iterator=readings;iterator>obstacle_element;iterator--){
            //         distance = // find distance of iterator from obstacle_element
            //         if(distance</*banana*/){
            //             end_index = iterator;// set new end = iterator
            //             end_found = true;
            //             break;
            //         }
            //     }
            // }
            if (obstacle_element==end_index){
                break;
            }
            obstacle_element++;
        }
        obstacle_indexes.push_back(end_index);
        std::cout<<":::: "<<end_index<<"\n";
    }
    return obstacle_indexes;
}

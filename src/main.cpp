#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

static float GetUserInput(){
    float input;
    std::cin >> input; 
    if(input <0 || input> 100){
      std::cout << "Value is invalid, please enter valid value" <<"\n";
      std::cin >> input;  
    }
    return input;
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    //Complete this TODO to satisfy Project Rubric Criterias of User Input
  
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    float start_x = 10;
    float start_y = 10;
    float end_x = 90;
    float end_y = 90;

    // user input for these values using std::cin. Pass the user input to the
    std::cout << "Please enter values from 0 to 100 for"
              << "\n";
    std::cout << "start_x\n";
    start_x = GetUserInput();

    std::cout << "start_y\n";
    start_y = GetUserInput();

    std::cout << "end_x\n";
    end_x = GetUserInput();

    std::cout << "end_y\n";
    end_y = GetUserInput();

    std::cout << "Entered values\nstart_x: " << start_x << "\nstart_y: " << start_y << "\nend_x: " << end_x << "\nend_y: " << end_y << "\n";


    // RoutePlanner object below in place of 10, 10, 90, 90.

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}

#include "system/MapStitch.hpp"

FILE *location_log = nullptr;

int main()
{
    MapStitch map_stitch;
    map_stitch.load_prior_map_info("/home/will/data/test_mapping/mapping1");
    map_stitch.load_stitch_map_info("/home/will/data/test_mapping/mapping2");
    return 0;
}

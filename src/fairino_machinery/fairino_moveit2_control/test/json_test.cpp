#include <iostream>
#include <nlohmann/json.hpp>

int main()
{
    std::string a = "{\"(0, 0)\": [0.000000, 0.000000, 0.000000]}";
    nlohmann::json j = nlohmann::json::parse(a);
    for (auto& [key_str, point_array] : j.items())
    {
        int x,y;
        if (sscanf(key_str.c_str(), "(%d, %d)", &x, &y) == 2) {
            std::cout<<x<<std::endl;
        }

        std::cout<<key_str<<std::endl;
        std::cout<<point_array[0]<<std::endl;

    }
}
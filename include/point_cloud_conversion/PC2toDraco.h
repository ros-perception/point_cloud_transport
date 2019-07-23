#ifndef POINT_CLOUD_TRANSPORT_PC2TODRACO_H
#define POINT_CLOUD_TRANSPORT_PC2TODRACO_H

// ros
#include <ros/ros.h> //
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>

// for enumeration in switch(std::string) ...
#include <map>
#include <string>

// for dynamic vector arrays
#include <vector>

// draco
#include <draco/point_cloud/point_cloud_builder.h>


class PC2toDraco {
public:
    //! Constructor.
    explicit PC2toDraco(sensor_msgs::PointCloud2 PC2);

    //! Destructor
    ~PC2toDraco();

    //! Method for converting into Draco pointcloud
    std::unique_ptr<draco::PointCloud> convert();

private:
    //! Message to be converted
    sensor_msgs::PointCloud2 PC2_;

    //! sensor_msgs::PointField::datatype names
    char *PointFieldDataTypes;

    //! enumeration for switch(std::string)
    enum StringValueFieldName { enumvalGeneric,
        enumval1,
        enumval2,
        enumval3,
        enumval4,
        enumval5,
        enumval6,
        enumval7,
        enumval8,
        enumval9,
        enumval10,
        enumval11,
        enumvalEnd };

    std::map<std::string, StringValueFieldName> s_mapStringValues;

    //! initialize enumeration values for detection of GeometryAttribute::Type
    void Initialize()
    {

        s_mapStringValues["x"] =        enumval1;
        s_mapStringValues["y"] =        enumval2;
        s_mapStringValues["z"] =        enumval3;
        s_mapStringValues["pos"] =      enumval4;
        s_mapStringValues["position"] = enumval5;
        s_mapStringValues["r"] =        enumval6;
        s_mapStringValues["g"] =        enumval7;
        s_mapStringValues["b"] =        enumval8;
        s_mapStringValues["a"] =        enumval9;
        s_mapStringValues["rgb"] =      enumval10;
        s_mapStringValues["rgba"] =     enumval11;
    }
};


#endif //POINT_CLOUD_TRANSPORT_PC2TODRACO_H

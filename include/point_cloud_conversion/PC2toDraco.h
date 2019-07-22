//
// Created by paplhjak@fel.cvut.cz on 7/10/19.
//

#ifndef PUBVEL_PC2TODRACO_H
#define PUBVEL_PC2TODRACO_H

#include <map>
#include <string>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <draco/point_cloud/point_cloud_builder.h>


class PC2toDraco {
public:
    //! Constructor.
    explicit PC2toDraco(sensor_msgs::PointCloud2 PC2);

    //! Method for converting into Draco Point Cloud
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

    //! initializes enumeration values for detecting GeometryAttribute::Type
    void Initialize()
    {
        //!Debug TODO delete
        std::cout<<"PC2toDraco object Initialize() called"<<std::endl;

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


#endif //PUBVEL_PC2TODRACO_H

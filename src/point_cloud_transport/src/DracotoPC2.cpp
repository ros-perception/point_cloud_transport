// TODO: unpack draco buffer into PointCloud2 format


#include "DracotoPC2.h"
#include "debug_msg.h"

//! Constructor
DracotoPC2::DracotoPC2(std::unique_ptr<draco::PointCloud> && pc, sensor_msgs::PointField fields[], uint32_t point_step) {
    pc_=(std::move(pc));
    fields_=fields;
    point_step_=point_step;
}

//! Destructor
DracotoPC2::~DracotoPC2() {}

//!
sensor_msgs::PointCloud2 DracotoPC2::DracotoPC2::convert(){

    //! number of all attributes of point cloud
    int32_t number_of_attributes = pc_->num_attributes();

    DBGVAR(std::cout, number_of_attributes);

    //!
    draco::PointIndex::ValueType number_of_points = pc_->num_points();

    DBGVAR(std::cout, number_of_points);

    //! Allocate memory for PointCloud2 data
    uint8_t data[number_of_points*point_step_];

    //! for each attribute
    for (uint32_t att_id = 0 ; att_id < number_of_attributes ; att_id++)
    {
        //! get attribute
        const draco::PointAttribute* attribute = pc_->GetAttributeByUniqueId(att_id);

        DBGVAR(std::cout, att_id);
        DBGVAR(std::cout, attribute->byte_stride());

        //! check if attribute is valid
        if (!attribute->IsValid()){
            //! RAISE ERROR - buffer of attribute is empty
            std::cout<< "\nAttribute not valid!\n" << std::endl;
        }

        //!
        uint32_t attribute_offset = fields_[att_id].offset;

        //! for each point in point cloud
        for (draco::PointIndex::ValueType point_index = 0; point_index < number_of_points; point_index++)
        {

            //DBGVAR(std::cout, int(point_step_*point_index + fields_[att_id].offset));

            //!
            uint8_t *out_data = &data[int(point_step_*point_index + fields_[att_id].offset)];

            //!
            attribute->GetValue(draco::AttributeValueIndex(point_index), out_data);

        }

        draco::DataType att_d_type = attribute->data_type();
        DBGVAR(std::cout, att_d_type);
    }
    //!

    std::cout << "FINISHED !!!" << std::endl;

    //!
    sensor_msgs::PointCloud2 PC2;

    DBGVAR(std::cout, 1);

    //PC2.fields[0].name = fields_[0].name;

    //DBGVAR(std::cout, PC2.fields[0].name);

    PC2.width = number_of_points;

    DBGVAR(std::cout, PC2.width);

    PC2.height = 1;

    std::vector<uint8_t> vec_data(data, data + number_of_points);

    DBGVAR(std::cout, 2);

    PC2.data = vec_data;

    DBGVAR(std::cout, PC2.data[0]);

    PC2.is_bigendian = false;

    DBGVAR(std::cout, PC2.is_bigendian);

    PC2.point_step = point_step_;

    DBGVAR(std::cout, PC2.point_step);

    PC2.row_step = PC2.width*point_step_;

    DBGVAR(std::cout, PC2.row_step);

    PC2.is_dense = false;

    DBGVAR(std::cout, PC2.is_dense);

    return std::move(PC2);

}

/*
enum DataType {
    // Not a legal value for DataType. Used to indicate a field has not been set.
            DT_INVALID = 0,
    DT_INT8,
    DT_UINT8,
    DT_INT16,
    DT_UINT16,
    DT_INT32,
    DT_UINT32,
    DT_INT64,
    DT_UINT64,
    DT_FLOAT32,
    DT_FLOAT64,
    DT_BOOL,
    DT_TYPES_COUNT
};
*/
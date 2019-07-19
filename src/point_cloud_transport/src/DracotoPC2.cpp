// TODO: work around copying unique_ptr, unpack draco buffer into PointCloud2 format


#include "DracotoPC2.h"
#include "debug_msg.h"

//! Constructor
DracotoPC2::DracotoPC2(std::unique_ptr<draco::PointCloud> pc, sensor_msgs::PointField field[]) {
    pc_ = pc;
    field_=field;

    const char *PointFieldDataTypes[] =
            {
                    "Undefined",
                    "INT8",
                    "UINT8",
                    "INT16",
                    "UINT16",
                    "INT32",
                    "UINT32",
                    "FLOAT32",
                    "FLOAT64"
            };
}

//!
sensor_msgs::PointCloud2 DracotoPC2::DracotoPC2::convert(){

    //! number of all attributes of point cloud
    int32_t number_of_attributes = pc_->num_attributes();

    //!
    draco::PointIndex::ValueType number_of_points = pc_->num_points();

    // for each attribute
    for (uint32_t att_id = 0 ; att_id < number_of_attributes ; att_id++)
    {
        const draco::PointAttribute* attribute = pc_->GetAttributeByUniqueId(att_id);

        //! check if attribute is valid
        if (!attribute->IsValid()){
            //! RAISE ERROR - buffer of attribute is empty

        }

        draco::DataType att_d_type = attribute->data_type();
    }
    //!


    /* for all attributes
     *  take the attribute
     *
     *
     *
     *
     */
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
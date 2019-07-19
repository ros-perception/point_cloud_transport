// TODO:

#ifndef PUBVEL_DRACOTOPC2_H
#define PUBVEL_DRACOTOPC2_H

#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include "draco/point_cloud/point_cloud.h"
#include <memory>

/*
 *
 * Point cloud methods:

  // Returns the number of named attributes of a given type.
  int32_t NumNamedAttributes(GeometryAttribute::Type type) const;

  // Returns attribute id of the first named attribute with a given type or -1
  // when the attribute is not used by the point cloud.
  int32_t GetNamedAttributeId(GeometryAttribute::Type type) const;

  // Returns the id of the i-th named attribute of a given type.
  int32_t GetNamedAttributeId(GeometryAttribute::Type type, int i) const;

  // Returns the first named attribute of a given type or nullptr if the
  // attribute is not used by the point cloud.
  const PointAttribute *GetNamedAttribute(GeometryAttribute::Type type) const;

  // Returns the i-th named attribute of a given type.
  const PointAttribute *GetNamedAttribute(GeometryAttribute::Type type,
                                          int i) const;

  // Returns the named attribute of a given unique id.
  const PointAttribute *GetNamedAttributeByUniqueId(
      GeometryAttribute::Type type, uint32_t id) const;

  // Returns the attribute of a given unique id.
  const PointAttribute *GetAttributeByUniqueId(uint32_t id) const;
  int32_t GetAttributeIdByUniqueId(uint32_t unique_id) const;

  int32_t num_attributes() const {
    return static_cast<int32_t>(attributes_.size());

    Attribute methods:



 *
 */


class DracotoPC2 {
public:
    //! Constructor.
    explicit DracotoPC2(std::unique_ptr<draco::PointCloud> && pc, sensor_msgs::PointField fields[], uint32_t point_step);

    //! Destructor
    ~DracotoPC2();

    //! Method for converting into sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 convert();

private:
    //! Message to be converted
    std::unique_ptr<draco::PointCloud> pc_;

    //!
    sensor_msgs::PointField* fields_;

    //!
    uint32_t point_step_;

};


#endif //PUBVEL_DRACOTOPC2_H

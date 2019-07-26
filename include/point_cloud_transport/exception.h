//
// Created by jakub on 7/25/19.
//

#ifndef POINT_CLOUD_TRANSPORT_EXCEPTION_H
#define POINT_CLOUD_TRANSPORT_EXCEPTION_H

#include <stdexcept>

namespace point_cloud_transport {

    //! A base class for all point_cloud_transport exceptions inheriting from std::runtime_error.
    class Exception : public std::runtime_error
    {
    public:
        Exception(const std::string& message) : std::runtime_error(message) {}
    };

    //! An exception class thrown when point_cloud_transport is unable to load a requested transport.
    class TransportLoadException : public Exception
    {
    public:
        TransportLoadException(const std::string& transport, const std::string& message)
                : Exception("Unable to load plugin for transport '" + transport + "', error string:\n" + message),
                  transport_(transport.c_str())
        {
        }

        std::string getTransport() const { return transport_; }

    protected:
        const char* transport_;
    };

} //namespace point_cloud_transport


#endif //POINT_CLOUD_TRANSPORT_EXCEPTION_H

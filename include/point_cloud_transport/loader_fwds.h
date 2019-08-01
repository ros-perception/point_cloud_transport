#ifndef POINT_CLOUD_TRANSPORT_LOADER_FWDS_H
#define POINT_CLOUD_TRANSPORT_LOADER_FWDS_H

// Forward-declare some classes most users shouldn't care about so that
// point_cloud_transport.h doesn't bring them in.

namespace pluginlib {
    template<class T> class ClassLoader;
}

namespace point_cloud_transport {
    class PublisherPlugin;
    class SubscriberPlugin;

    typedef pluginlib::ClassLoader<PublisherPlugin> PubLoader;
    typedef boost::shared_ptr<PubLoader> PubLoaderPtr;

    typedef pluginlib::ClassLoader<SubscriberPlugin> SubLoader;
    typedef boost::shared_ptr<SubLoader> SubLoaderPtr;
}

#endif //POINT_CLOUD_TRANSPORT_LOADER_FWDS_H

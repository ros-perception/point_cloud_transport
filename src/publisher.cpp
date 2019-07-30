#include "point_cloud_transport/publisher.h"
#include "point_cloud_transport/publisher_plugin.h"
#include <pluginlib/class_loader.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/erase.hpp>

namespace point_cloud_transport {

    struct Publisher::Impl
    {
        Impl()
                : unadvertised_(false)
        {
        }

        ~Impl()
        {
            shutdown();
        }

        uint32_t getNumSubscribers() const
        {
            uint32_t count = 0;
            BOOST_FOREACH(const boost::shared_ptr<PublisherPlugin>& pub, publishers_)
            count += pub->getNumSubscribers();
            return count;
        }

        std::string getTopic() const
        {
            return base_topic_;
        }

        bool isValid() const
        {
            return !unadvertised_;
        }

        void shutdown()
        {
            if (!unadvertised_) {
                unadvertised_ = true;
                BOOST_FOREACH(boost::shared_ptr<PublisherPlugin>& pub, publishers_)
                pub->shutdown();
                publishers_.clear();
            }
        }

        void subscriberCB(const SingleSubscriberPublisher& plugin_pub,
                          const SubscriberStatusCallback& user_cb)
        {
            SingleSubscriberPublisher ssp(plugin_pub.getSubscriberName(), getTopic(),
                                          boost::bind(&Publisher::Impl::getNumSubscribers, this),
                                          plugin_pub.publish_fn_);
            user_cb(ssp);
        }

        std::string base_topic_;
        PubLoaderPtr loader_;
        std::vector<boost::shared_ptr<PublisherPlugin> > publishers_;
        bool unadvertised_;
    };

    //! Constructor
    Publisher::Publisher(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                         const SubscriberStatusCallback& connect_cb,
                         const SubscriberStatusCallback& disconnect_cb,
                         const ros::VoidPtr& tracked_object, bool latch,
                         const PubLoaderPtr& loader)
            : impl_(new Impl)
    {
        // Resolve the name explicitly because otherwise the compressed topics don't remap properly
        impl_->base_topic_ = nh.resolveName(base_topic);
        impl_->loader_ = loader;

        // sequence container which encapsulates dynamic size arrays
        std::vector<std::string> blacklist_vec;
        // set
        std::set<std::string> blacklist;

        // call to parameter server
        nh.getParam(impl_->base_topic_ + "/disable_pub_plugins", blacklist_vec);
        for (size_t i = 0; i < blacklist_vec.size(); ++i)
        {
            blacklist.insert(blacklist_vec[i]);
        }

        BOOST_FOREACH(const std::string& lookup_name, loader->getDeclaredClasses()) {
            const std::string transport_name = boost::erase_last_copy(lookup_name, "_pub");
            if (blacklist.count(transport_name)) // searches the container for elements equivalent to val and returns the number of matches.
            {
                continue;
            }

            try {
                boost::shared_ptr<PublisherPlugin> pub = loader->createInstance(lookup_name);
                impl_->publishers_.push_back(pub);
                pub->advertise(nh, impl_->base_topic_, queue_size, rebindCB(connect_cb),
                               rebindCB(disconnect_cb), tracked_object, latch);
            }
            catch (const std::runtime_error& e) {
                ROS_DEBUG("Failed to load plugin %s, error string: %s",
                          lookup_name.c_str(), e.what());
            }
        }

        if (impl_->publishers_.empty())
            throw Exception("No plugins found! Does `rospack plugins --attrib=plugin "
                            "point_cloud_transport` find any packages?");
    }

    //!
    uint32_t Publisher::getNumSubscribers() const
    {
        if (impl_ && impl_->isValid()) return impl_->getNumSubscribers();
        return 0;
    }

    //!
    std::string Publisher::getTopic() const
    {
        if (impl_) return impl_->getTopic();
        return std::string();
    }

    //!
    void Publisher::publish(const sensor_msgs::PointCloud2& message) const
    {
        if (!impl_ || !impl_->isValid()) {
            ROS_ASSERT_MSG(false, "Call to publish() on an invalid point_cloud_transport::Publisher");
            return;
        }

        BOOST_FOREACH(const boost::shared_ptr<PublisherPlugin>& pub, impl_->publishers_) {
            if (pub->getNumSubscribers() > 0)
                pub->publish(message);
        }
    }

    //!
    void Publisher::publish(const sensor_msgs::PointCloud2ConstPtr& message) const
    {
        if (!impl_ || !impl_->isValid()) {
            ROS_ASSERT_MSG(false, "Call to publish() on an invalid point_cloud_transport::Publisher");
            return;
        }

        BOOST_FOREACH(const boost::shared_ptr<PublisherPlugin>& pub, impl_->publishers_) {
            if (pub->getNumSubscribers() > 0)
                pub->publish(message);
        }
    }

    //!
    void Publisher::shutdown()
    {
        if (impl_) {
            impl_->shutdown();
            impl_.reset();
        }
    }

    //!
    Publisher::operator void*() const
    {
        return (impl_ && impl_->isValid()) ? (void*)1 : (void*)0;
    }

    //!
    void Publisher::weakSubscriberCb(const ImplWPtr& impl_wptr,
                                     const SingleSubscriberPublisher& plugin_pub,
                                     const SubscriberStatusCallback& user_cb)
    {
        if (ImplPtr impl = impl_wptr.lock())
            impl->subscriberCB(plugin_pub, user_cb);
    }

    //!
    SubscriberStatusCallback Publisher::rebindCB(const SubscriberStatusCallback& user_cb)
    {
        // Note: the subscriber callback must be bound to the internal Impl object, not
        // 'this'. Due to copying behavior the Impl object may outlive the original Publisher
        // instance. But it should not outlive the last Publisher, so we use a weak_ptr.
        if (user_cb)
        {
            ImplWPtr impl_wptr(impl_);
            return boost::bind(&Publisher::weakSubscriberCb, impl_wptr, _1, user_cb);
        }
        else
            return SubscriberStatusCallback();
    }
} // namespace point_cloud_transport
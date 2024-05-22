#ifndef EZBOT_PROPULSION_GAZEBO_PLUGIN_HPP_
#define GAZEBO_OMNI_DRIVE_HPP_

#include <ezbot_propulsion_gazebo_plugin_private.hpp>
#include <gazebo/common/Plugin.hh>
#include <memory>

namespace ezbot_propulsion_gazebo_plugin
{
class ezbotPropulsionGazeboPlugin : public gazebo::ModelPlugin
{
  public:
    /// Constructor
    ezbotPropulsionGazeboPlugin();
  
    /// Destructor
    ~ezbotPropulsionGazeboPlugin();
  
  protected:
    // Documentation inherited
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    // Documentation inherited
    void Reset() override;
  private:
  /// Private data pointer
    std::unique_ptr<ezbotPropulsionGazeboPluginPrivate> impl_;
};
}  // namespace gazebo_plugins



#endif
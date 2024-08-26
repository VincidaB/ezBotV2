#include "gz_match_plugin/gz_match_plugin.hpp"

#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include "gz_match_plugin/team.hpp"

#include "gz_match_plugin/game_action.hpp"
#include "gz_match_plugin/action_zones.hpp"

using namespace gz;
using namespace sim;
using namespace systems;

// Inherit from System and 2 extra interfaces:
// ISystemConfigure and ISystemPostUpdate
class MyPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
{
  // Implement Configure callback, provided by ISystemConfigure
  // and called once at startup.
  virtual void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventMgr*/) override
  {
    // Create model object to access convenient functions
    world = World(_entity);

    action.setWorld(world);
    action.updateStart();


  }
 
  // Implement PostUpdate callback, provided by ISystemPostUpdate
  // and called at every iteration, after physics is done
  virtual void PostUpdate(const UpdateInfo &/*_info*/,
                          const EntityComponentManager &_ecm) override
  {
    //action.updateStart();

    
		    // Get model entity
    Entity modelEntity = world.ModelByName(_ecm, "SolarPanel_0");
    if (modelEntity == kNullEntity)
    {
      std::cerr << "Model 'SolarPanel_0' not found" << std::endl;
      return;
    }
		
		// Create model object
    Model model(modelEntity);

		linkEntity = model.LinkByName(_ecm, "solar_panel");
		if (linkEntity == kNullEntity)
    {
      std::cerr << "Link 'solar_panel' not found" << std::endl;
      return;
    }

    action.update(world, _ecm);
		
		//std::cout << worldPose(this->linkEntity, _ecm) << std::endl;
		//std::cout << "# of models : " << world.ModelCount(_ecm) << std::endl;

  }
 
  // ID of link entity
  private: Entity linkEntity;
	private: World world;

  private:ActionZones action = ActionZones();
};
 
// Register plugin
GZ_ADD_PLUGIN(MyPlugin,
                    gz::sim::System,
                    MyPlugin::ISystemConfigure,
                    MyPlugin::ISystemPostUpdate)
 
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(MyPlugin, "gz::sim::systems::MyPlugin")
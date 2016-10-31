# Providers #

The provider system exists to make refactoring the program and creating new objects as simple as possible. The factory methods inside each provider should contain as few parameters as possible, but as many as needed.

## FileProvider.java ##

Provides factory methods for Java objects that are created based on the contents of files, such as generic Java configuration files or JSON files. The constructed objects should not simply be the raw object that e.g. JSON parsers produce, but specialized objects that provide the exact structure of the information contained in files used by this program.

## RosServiceProvider.java ##

Provides understandably named factory methods for all the ROS Services needed by the program. Does not depend on other providers.

## RosServerProvider.java ##

Provides understandably named factory methods for all the ROS Server needed by the program. Does not depend on other providers.

## RosSubscriberProvider.java ##

Provides understandably named factory methods for all the ROS Subscribers needed by the program. Does not depend on other providers.
## Ros PublisherProvider.java ##

Provides understandably named factory methods for al the ROS Publishers needed by the program. Does not depend on other providers.

## RosParamProvider.java ##

Provides understandably named factory methods for the ros parameters needed by the program. Does not depend on other providers.

## ActionProvider.java ##

Provides factory methods for all the actions used by the program. Hides the actions' dependencies on services and subscribers from the creator of the actions. Depends on RosServiceProvider, RosSubscriberProvider and DroneStateTracker to create the actions.

## StateProvider.java ##

Provides factory methods for all the states used by the program. Hides the states' dependencies on services and actions from the creator of the states. Depends on RosServiceProvider and ActionProvider to create the states.

# Scripting #

## Script JSON format ##

> {
> "queue":[
> {
>         "state":"ScriptedState",
>         "scriptedActions":[
> {
 >        "action":"ArmAction"
> },
> {
 >      "action":"TakeOffAction",
 >       "target_heightcm":500
>},
>{
>        "action":"LandingAction"
>}
>]
>}
>]
>}

The JSON object contains the property "queue", which is an array filled with objects that represent states. The uppermost state is considered the first and vice versa. Each state object has the property "state" which is the name of the state and the named parameters the state needs (the array "scriptedActions" for ScriptedState and the array "waypoints" for WaypointState).

## Script Java format ##

> private class JsonStateList {
>     List<StateJsonRepresentation> queue;
>  }
>
> private class StateJsonRepresentation {
>     String state;
>     List<ActionJsonRepresentation> scriptedActions;
>     long duration;
>     //other types of parameters go here by name
> }

FlightScriptParser.java reads the JSON into a StringBuffer and uses the Gson library to transform it into a JsonStateList object. In Java terms, JSON arrays are Lists and JSON objects are classes.  Obviously, JSON Strings are Strings, JSON Numbers are Java Doubles and so on.

A quirk of the system is how StateJsonRepresentation contains properties for the parameters of each and every state. This is because Java's type system is not expressive enough to recognize variants of a common StateJsonRepresentation. Instead, the parser has to pretend that each state defined in JSON is an instance of a single superstate, where most of the fields are simply left undefined. The same goes for ActionJsonRepresentation.

## Adding support for a new state in the scripting system ##

*Note: This assumes the new state is already added to the program itself. The guide to adding new states can be found in states.md*

1. Add the script-defined parameters of the state as fields of StateJsonRepresentation. If the new state's parameters are JSON objects, you will have to create a private class (much like StateJsonRepresentation) to represent those objects. The Gson library will then automatically read the JSON state parameter objects as the Java object representation defined by you, accessible through the field you added to StateJsonRepresentation.
2. Add your state's name to the switch block of the parseState method of FlightScriptParser.
3. Inside your state's block inside the switch clause, convert the parameter information contained in your state's StateJsonRepresentation to the Java object(s) which your state's factory method (within StateProvider.java) takes. Return an instance of your state created by a method call to the StateProvider instance contained in FlightScriptParser.

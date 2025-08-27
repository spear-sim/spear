import numpy as np

# LineTraceSingle c++ header
'''
bool UKismetSystemLibrary::LineTraceSingle(
    const UObject* WorldContextObject,
    const FVector Start,
    const FVector End,
    ETraceTypeQuery TraceChannel,
    bool bTraceComplex,
    const TArray<AActor*>& ActorsToIgnore, 
    EDrawDebugTrace::Type DrawDebugType,
    FHitResult& OutHit,
    bool bIgnoreSelf,
    FLinearColor TraceColor, 
    FLinearColor TraceHitColor,
    float DrawTime
)
'''
def LineTraceSingle(
            game,
            start,
            end,
            trace_channel = 'ECC_WorldStatic', #
            b_trace_complex = False, #
            actors_to_ignore = [],
            draw_debug_type = 'None', #
            b_ignore_self = True, #
            trace_color = (1.0, 0.0, 0.0),
            trace_hit_color = (0.0, 1.0, 0.0),
            draw_time = 99999.0
        ):
    
    ## get pointer to unreal function
    # get kismet library
    kismet_static_class = game.unreal_service.get_static_class(class_name="UKismetSystemLibrary")
    kismet_default_object = game.unreal_service.get_default_object(uclass=kismet_static_class, create_if_needed=False)

    # get function pointer    
    line_trace_func = game.unreal_service.find_function_by_name(uclass=kismet_static_class, function_name="LineTraceSingle")

    # Prepare input args
    ## convert python args into dicts that SPEAR will parse into Unreal Structs
    start = {"X": start[0], "Y": start[1], "Z": start[2]}
    end = {"X": end[0], "Y": end[1], "Z": end[2]}
    trace_color = {'R': 1.0, 'G': 0.0, 'B': 0.0, 'A': 1.0}
    trace_hit_color = {'R': 0.0, 'G': 1.0, 'B': 0.0, 'A': 1.0}

    ## access Unreal enums w/ string name
    trace_channel = 'TraceTypeQuery1' 
    # TODO: ^ this should really be 'ECC_WorldStatic' 
    # TODO: currently it's getting converted into that iside LineTrace, but this mapping could change for different unreal project
    # TODO: not sure how to find the mapping dynamically, which would be the better solution
    
    ## prepare args
    args = { # const UObject* WorldContextObject, # Not needed, provided by SPEAR
        "Start": start,
        "End": end,
        "TraceChannel": trace_channel,
        "bTraceComplex": b_trace_complex,
        "ActorsToIgnore": actors_to_ignore,
        "DrawDebugType": draw_debug_type,
        "OutHit": {},
        "bIgnoreSelf": b_ignore_self,
        "TraceColor": trace_color,
        "TraceHitColor": trace_hit_color,
        "DrawTime": draw_time
    }

    ## call the function
    return_values = game.unreal_service.call_function(uobject=kismet_default_object, ufunction=line_trace_func, args=args)


    # Break out some convenient return values
    ## did the ray hit something?
    hit_something = return_values['ReturnValue']

    ## where did it hit?
    ray_impact = None
    if hit_something:
        ray_impact = return_values['OutHit']['impactPoint']
        ray_impact = np.array([ray_impact['x'], ray_impact['y'], ray_impact['z']])


    # Return
    return return_values, hit_something, ray_impact

# SphereOverlapActors c++ header
'''
bool UKismetSystemLibrary::SphereOverlapActors(
    const UObject* WorldContextObject,
    const FVector SpherePos,
    float SphereRadius,
    const TArray<TEnumAsByte<EObjectTypeQuery> > & ObjectTypes,
    UClass* ActorClassFilter,
    const TArray<AActor*>& ActorsToIgnore,
    TArray<AActor*>& OutActors)
'''
def SphereOverlapActors(
                        game,
                        sphere_pos,
                        sphere_radius,
                        object_types = [f'ObjectTypeQuery{i}' for i in range(1,7)], # N.B. TEnumAsByte is handled internally, just pass enum values as strings in list
                        actor_class_filter = None,
                        actors_to_ignore = []
                    ):

    ## get pointer to unreal function
    # get kismet library
    kismet_static_class = game.unreal_service.get_static_class(class_name="UKismetSystemLibrary")
    kismet_default_object = game.unreal_service.get_default_object(uclass=kismet_static_class, create_if_needed=False)

    # get function pointer
    sphere_overlap_actors_func = game.unreal_service.find_function_by_name(uclass=kismet_static_class, function_name="SphereOverlapActors")

    # Prepare input args
    ## convert python args into dicts that SPEAR will parse into Unreal Structs
    sphere_pos = {"X": sphere_pos[0], "Y": sphere_pos[1], "Z": sphere_pos[2]}

    ## prepare args
    args = { # const UObject* WorldContextObject, # Not needed, provided by SPEAR in unreal_service.call_function
        "SpherePos": sphere_pos,
        "SphereRadius": sphere_radius,
        "ObjectTypes": object_types,
        #"ActorClassFilter": actor_class_filter,
        "ActorsToIgnore": actors_to_ignore,
        "OutActors": [],
    }

    ## call the function
    return_values = game.unreal_service.call_function(uobject=kismet_default_object, ufunction=sphere_overlap_actors_func, args=args)

    # Break out some convenient return values
    ## did the sphere overlap any actors?
    overlapped_something = return_values['ReturnValue']

    ## what actors did it overlap?
    out_actors = []
    if overlapped_something:
        out_actors = return_values['OutActors']

    # Return
    return return_values, overlapped_something, out_actors

# SphereTraceSingle c++ header
'''
bool SphereTraceSingle(
    const UObject* WorldContextObject,
    const FVector Start,
    const FVector End,
    float Radius,
    ETraceTypeQuery TraceChannel,
    bool bTraceComplex,
    const TArray<AActor*>& ActorsToIgnore,
    EDrawDebugTrace::Type DrawDebugType,
    FHitResult& OutHit,
    bool bIgnoreSelf,
    FLinearColor TraceColor = FLinearColor::Red,
    FLinearColor TraceHitColor = FLinearColor::Green,
    float DrawTime = 5.0f);
'''
def SphereTraceSingle(
            game,
            start,
            end,
            radius,
            trace_channel = 'ECC_WorldStatic',
            b_trace_complex = False,
            actors_to_ignore = [],
            draw_debug_type = 'None',
            b_ignore_self = True,
            trace_color = (1.0, 0.0, 0.0),
            trace_hit_color = (0.0, 1.0, 0.0),
            draw_time = 5.0
        ):

    ## get pointer to unreal function
    # get kismet library
    kismet_static_class = game.unreal_service.get_static_class(class_name="UKismetSystemLibrary")
    kismet_default_object = game.unreal_service.get_default_object(uclass=kismet_static_class, create_if_needed=False)

    # get function pointer
    sphere_trace_func = game.unreal_service.find_function_by_name(uclass=kismet_static_class, function_name="SphereTraceSingle")

    # Prepare input args
    ## convert python args into dicts that SPEAR will parse into Unreal Structs
    start = {"X": start[0], "Y": start[1], "Z": start[2]}
    end = {"X": end[0], "Y": end[1], "Z": end[2]}
    trace_color = {'R': trace_color[0], 'G': trace_color[1], 'B': trace_color[2], 'A': 1.0}
    trace_hit_color = {'R': trace_hit_color[0], 'G': trace_hit_color[1], 'B': trace_hit_color[2], 'A': 1.0}

    ## access Unreal enums w/ string name
    trace_channel = 'TraceTypeQuery1'
    # TODO: ^ this should really be 'ECC_WorldStatic'
    # TODO: currently it's getting converted into that iside LineTrace, but this mapping could change for different unreal project
    # TODO: not sure how to find the mapping dynamically, which would be the better solution

    ## prepare args
    args = { # const UObject* WorldContextObject, # Not needed, provided by SPEAR
        "Start": start,
        "End": end,
        "Radius": radius,
        "TraceChannel": trace_channel,
        "bTraceComplex": b_trace_complex,
        "ActorsToIgnore": actors_to_ignore,
        "DrawDebugType": draw_debug_type,
        "OutHit": {},
        "bIgnoreSelf": b_ignore_self,
        "TraceColor": trace_color,
        "TraceHitColor": trace_hit_color,
        "DrawTime": draw_time
    }

    ## call the function
    return_values = game.unreal_service.call_function(uobject=kismet_default_object, ufunction=sphere_trace_func, args=args)

    # Break out some convenient return values
    ## did the ray hit something?
    hit_something = return_values['ReturnValue']

    ## where did it hit?
    ray_impact = None
    if hit_something:
        ray_impact = return_values['OutHit']['impactPoint']
        ray_impact = np.array([ray_impact['x'], ray_impact['y'], ray_impact['z']])

    # Return
    return return_values, hit_something, ray_impact
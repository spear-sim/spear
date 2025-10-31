//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <AI/Navigation/NavigationTypes.h> // FNavDataConfig, FNavLocation
#include <Kismet/BlueprintFunctionLibrary.h>
#include <NavFilters/NavigationQueryFilter.h>
#include <NavigationData.h>
#include <Math/Vector.h>
#include <Templates/SubclassOf.h>

#include "SpCore/Assert.h"

#include "SpNavigationData.generated.h"

UCLASS()
class USpNavigationData : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public: 
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FNavDataConfig GetConfig(const ANavigationData* NavigationData)
    {
        SP_ASSERT(NavigationData);
        return NavigationData->GetConfig();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FVector GetRandomPoint(ANavigationData* NavigationData, TSubclassOf<UNavigationQueryFilter> FilterClass = NULL)
    {
        SP_ASSERT(NavigationData);
        return NavigationData->GetRandomPoint(UNavigationQueryFilter::GetQueryFilter(*NavigationData, nullptr, FilterClass)).Location;
    }
};

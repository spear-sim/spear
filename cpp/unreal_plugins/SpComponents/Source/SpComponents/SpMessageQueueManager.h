//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <deque>

#include <Containers/UnrealString.h> // FString
#include <GameFramework/Actor.h>
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UPROPERTY

#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpMessageQueueManager.generated.h"

UCLASS()
class ASpMessageQueueManager : public AActor
{
    GENERATED_BODY()
public: 
    ASpMessageQueueManager()
    {
        SP_LOG_CURRENT_FUNCTION();
    };

    ~ASpMessageQueueManager() override
    {
        SP_LOG_CURRENT_FUNCTION();
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void CreateQueue(FString queue_name)
    {
        Std::insert(message_queues_, Unreal::toStdString(queue_name), {});
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void DestroyQueue(FString queue_name)
    {
        Std::remove(message_queues_, Unreal::toStdString(queue_name));
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    int GetQueueLength(FString queue_name)
    {
        return message_queues_.at(Unreal::toStdString(queue_name)).size();
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void PushMessageToFrontOfQueue(FString queue_name, FString message)
    {
        message_queues_.at(Unreal::toStdString(queue_name)).push_front(Unreal::toStdString(message));
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void PushMessageToBackOfQueue(FString queue_name, FString message)
    {
        message_queues_.at(Unreal::toStdString(queue_name)).push_back(Unreal::toStdString(message));
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    FString PopMessageFromFrontOfQueue(FString queue_name)
    {
        std::deque<std::string>& queue = message_queues_.at(Unreal::toStdString(queue_name));
        std::string str = queue.front();
        queue.pop_front();
        return Unreal::toFString(str);
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    FString PopMessageFromBackOfQueue(FString queue_name)
    {
        std::deque<std::string>& queue = message_queues_.at(Unreal::toStdString(queue_name));
        std::string str = queue.front();
        queue.pop_front();
        return Unreal::toFString(str);
    };

private:
    std::map<std::string, std::deque<std::string>> message_queues_;
};

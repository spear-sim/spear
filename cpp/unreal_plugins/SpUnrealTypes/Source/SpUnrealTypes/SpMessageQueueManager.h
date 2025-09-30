//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <deque>

#include <Containers/UnrealString.h> // FString
#include <UObject/Object.h>          // UObject
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UPROPERTY

#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpMessageQueueManager.generated.h"

UCLASS()
class USpMessageQueueManager : public UObject
{
    GENERATED_BODY()
public: 
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void CreateQueue(FString QueueName)
    {
        Std::insert(message_queues_, Unreal::toStdString(QueueName), {});
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void DestroyQueue(FString QueueName)
    {
        Std::remove(message_queues_, Unreal::toStdString(QueueName));
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    bool HasQueue(FString QueueName) const
    {
        return Std::containsKey(message_queues_, Unreal::toStdString(QueueName));
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    int GetQueueLength(FString QueueName) const
    {
        return message_queues_.at(Unreal::toStdString(QueueName)).size();
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    FString GetMessage(FString QueueName, int Num) const
    {
        return Unreal::toFString(message_queues_.at(Unreal::toStdString(QueueName)).at(Num));
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void PushMessageToFrontOfQueue(FString QueueName, FString Message)
    {
        message_queues_.at(Unreal::toStdString(QueueName)).push_front(Unreal::toStdString(Message));
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void PushMessageToBackOfQueue(FString QueueName, FString Message)
    {
        message_queues_.at(Unreal::toStdString(QueueName)).push_back(Unreal::toStdString(Message));
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    FString PopMessageFromFrontOfQueue(FString QueueName)
    {
        std::deque<std::string>& queue = message_queues_.at(Unreal::toStdString(QueueName));
        SP_ASSERT(!queue.empty());
        std::string str = queue.front();
        queue.pop_front();
        return Unreal::toFString(str);
    };

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    FString PopMessageFromBackOfQueue(FString QueueName)
    {
        std::deque<std::string>& queue = message_queues_.at(Unreal::toStdString(QueueName));
        SP_ASSERT(!queue.empty());
        std::string str = queue.back();
        queue.pop_back();
        return Unreal::toFString(str);
    };

private:
    // TMap<FString, TArray<...>> is not supported as a UPROPERTY type, so we declare a private
    // member variable and provide accessor functions.
    std::map<std::string, std::deque<std::string>> message_queues_;
};

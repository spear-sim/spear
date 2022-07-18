#include "CoreMinimal.h"
#include "UObject/Object.h"

#include "ObjectLoader.generated.h"


UCLASS()
template<class T>
class UObjectLoader : public UObject
{
	GENERATED_BODY()
		
public:
	UObjectLoader(const FObjectInitializer &ObjectInitializer, const FString &Path ){
        FString type = FindAssetType(Path);
        switch (type)
        {
        case "Material":
            /* code */
            break;
        
        default:
            UE_LOG(LogTemp, Warning, TEXT("asset load error. type not included."));
            break;
        }
    }

private:
    FString FindAssetType(const FString &Path){
        return FString("hi");
    }

    T* loaded_object;
    
};
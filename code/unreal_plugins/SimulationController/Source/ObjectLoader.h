#include "CoreMinimal.h"
#include "UObject/Object.h"

#include "ObjectLoader.generated.h"


UCLASS()
class UObjectLoader : public UObject
{
	GENERATED_BODY()
		
public:
    UObjectLoader(){}
	//UObjectLoader(const FObjectInitializer &ObjectInitializer, const FString &Path ){
    //    FString type = FindAssetType(Path);
    //    switch (type)
    //    {
    //    case "Material":
    //        ConstructorHelpers::FObjectFinder<UMaterial> Loader(*Path);
  	//        if (Loader.Succeeded()){
    //	        this->loaded_object = Loader.Object;
  	//        }else{
    //            UE_LOG(LogTemp, Warning, TEXT("Error on loading asset."));
    //        }
    //        break;
    //    
    //    default:
    //        UE_LOG(LogTemp, Warning, TEXT("asset load error. type not included."));
    //        break;
    //    }
    //}
    //
    //T* GetLoadedObject(){
    //    return this->loaded_object;
    //}

private:
    FString FindAssetType(const FString &Path){
        //Path.Find();
        return FString("hi");
    }

    //T* loaded_object;
    
};
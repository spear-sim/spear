#include "SimpleVehicleSimApi.h"

SimpleVehicleSimApi::SimpleVehicleSimApi(Params params): RobotSimApiBase(params)
{
    this->createVehicleApi(
        static_cast<ASimpleVehiclePawn*>(params.vehicle->GetPawn()),
        params.home_geopoint);
}

void SimpleVehicleSimApi::updateRenderedState(float dt)
{
    RobotSimApiBase::updateRenderedState(dt);

    // TODO: What does this do?
    this->vehicle_api_->getStatusMessages(this->vehicle_api_messages_);
}

void SimpleVehicleSimApi::updateRendering(float dt)
{
    RobotSimApiBase::updateRendering(dt);

    for (auto message : this->vehicle_api_messages_)
    {
        URobotBlueprintLib::LogMessage(FString(message.c_str()), TEXT(""),
                                       LogDebugLevel::Success, 30);
    }

    try
    {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception& e)
    {
        URobotBlueprintLib::LogMessage(FString(e.what()), TEXT(""),
                                       LogDebugLevel::Failure, 30);
    }
}

void SimpleVehicleSimApi::reset()
{
    RobotSimApiBase::reset();
    this->vehicle_api_->reset();
}

void SimpleVehicleSimApi::update()
{
    this->vehicle_api_->update();
    RobotSimApiBase::update();
}

void SimpleVehicleSimApi::reportState(RobotSim::StateReporter& reporter)
{
    // TODO: Implementation copied from car. We should probably report more
    // stuff?

    // report actual location in unreal coordinates so we can plug that into the
    // UE editor to move the drone.
    FVector unrealPosition = this->getUUPosition();
    reporter.writeValue(
        "unreal pos",
        Vector3r(unrealPosition.X, unrealPosition.Y, unrealPosition.Z));
}

std::string SimpleVehicleSimApi::getRecordFileLine(bool is_header_line) const
{
    // TODO: Do something acutally useful here.
    return "Not implemented yet. Whee!\n";
}

void SimpleVehicleSimApi::createVehicleApi(
    ASimpleVehiclePawn* pawn, const RobotSim::GeoPoint& home_geopoint)
{
    this->vehicle_api_ =
        std::unique_ptr<SimpleVehicleApi>(new SimpleVehicleApi());
}

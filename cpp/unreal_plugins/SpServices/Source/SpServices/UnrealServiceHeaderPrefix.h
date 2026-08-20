//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

// We place these includes before all other Unreal includes to guarantee that they are aren't affected by the
// include order of other Unreal headers.

// MessageLog is a Developer-category module and is therefore not available in Shipping builds.
#if WITH_UNREAL_DEVELOPER_TOOLS

    //
    // HACK: FMessageLogModule::MessageLogViewModel and FMessageLogListingViewModel::MessageLogListingModel are
    // both private members, and there is no public API for enumerating all registered message log listings, or
    // for reading all (as opposed to filtered) messages across all pages of a listing. We override private
    // visibility in the headers below, but only ever reach into these two members directly. Reading a private
    // member needs no exported symbol, since it's just a memory offset, not a function call, so this links
    // correctly. 
    //
    // Every method we call from the headers below is either inline (FMessageLogViewModel::GetLogListingViewModels())
    // or a member of a class that is properly exported from the MessageLog module (FMessageLogListingViewModel and
    // FMessageLogListingModel are both MESSAGELOG_API), so this links correctly. FMessageLogViewModel and
    // FMessageLogModule themselves are not exported, so we deliberately avoid calling any of their non-inline
    // methods (e.g., FindLogListingViewModel(...)). We must include Presentation/MessageLogListingViewModel.h
    // under this #define too (not just MessageLogModule.h), because Presentation/MessageLogViewModel.h
    // transitively includes it. If that transitive include happens first, outside this block, its header guard
    // would make our own include below a no-op, leaving MessageLogListingModel private after all.
    //

    #pragma push_macro("private")
    #undef private
    #define private public
        #include <Presentation/MessageLogListingViewModel.h> // FMessageLogListingViewModel
        #include <Presentation/MessageLogViewModel.h>        // FMessageLogViewModel
        #include <MessageLogModule.h>                        // FMessageLogModule
    #pragma pop_macro("private")

#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "PhysicsBody.hpp"
#include "common_utils/Common.hpp"
#include "common_utils/UpdatableContainer.hpp"

namespace RobotSim {
    /**
     * @brief
     *
     */
    class PhysicsEngineBase : public UpdatableObject {
    public:
        /**
         * @brief
         *
         */
        virtual void reset() override
        {
            UpdatableObject::reset();
        }

        /**
         * @brief
         *
         */
        virtual void update() override
        {
            UpdatableObject::update();
        }

        /**
         * @brief
         *
         * @param reporter
         */
        virtual void reportState(StateReporter& reporter) override
        {
            unused(reporter);
            // default nothing to report for physics engine
        }

        // TODO: reduce copy-past from UpdatableContainer which has same code
        /********************** Container interface **********************/
        typedef PhysicsBody* TUpdatableObjectPtr;
        typedef vector<TUpdatableObjectPtr> MembersContainer;
        typedef typename MembersContainer::iterator iterator;
        typedef typename MembersContainer::const_iterator const_iterator;
        typedef typename MembersContainer::value_type value_type;

        iterator begin() { return members_.begin(); }

        /**
         * @brief
         *
         * @return iterator
         */
        iterator end() { return members_.end(); }

        /**
         * @brief
         *
         * @return const_iterator
         */
        const_iterator begin() const { return members_.begin(); }

        /**
         * @brief
         *
         * @return const_iterator
         */
        const_iterator end() const { return members_.end(); }

        /**
         * @brief
         *
         * @return uint
         */
        uint size() const { return static_cast<uint>(members_.size()); }

        /**
         * @brief
         *
         * @param index
         * @return const TUpdatableObjectPtr&
         */
        const TUpdatableObjectPtr& at(uint index) const { return members_.at(index); }

        /**
         * @brief
         *
         * @param index
         * @return TUpdatableObjectPtr&
         */
        TUpdatableObjectPtr& at(uint index) { return members_.at(index); }

        /**
         * @brief Allows overriding membership modifications
         *
         */
        virtual void clear() { members_.clear(); }

        /**
         * @brief
         *
         * @param member
         */
        virtual void insert(TUpdatableObjectPtr member) { members_.push_back(member); }

        /**
         * @brief
         *
         * @param obj
         */
        virtual void erase_remove(TUpdatableObjectPtr obj) { members_.erase(std::remove(members_.begin(), members_.end(), obj), members_.end()); }

    private:
        MembersContainer members_;
    };

} // namespace

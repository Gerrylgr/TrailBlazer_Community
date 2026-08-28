/*
*   Critic 基类
*/
#ifndef LOCAL_PLANNER__MPPI__CORE__CRITIC_FUNCTION_HPP_
#define LOCAL_PLANNER__MPPI__CORE__CRITIC_FUNCTION_HPP_

#include "local_planner/MPPI/core/critic_data.hpp"

namespace local_planner::mppi_core
{

    class CriticFunction
    {
    public:
        // 多态：这个纯虚函数在编译时不会确认真正的类，只有到真正执行时才会确认要具体执行哪个子类的方法
        virtual ~CriticFunction() = default;

        virtual void score(CriticData & data) const = 0;
    };

}  // namespace local_planner::mppi_core

#endif
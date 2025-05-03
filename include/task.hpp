#pragma once

#include <stdexcept>
#include <functional>
#include <string_view>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace eventide
{

class task
{
public:

    task()
    {}

    template <typename F, typename... Args>
    task(std::string_view name, F&& f, Args&&... args)
    {
        init(f);
    }

    ~task()
    {
        vTaskDelete(_handle);
    }

    template <typename F, typename... Args>
    void init(std::string_view name, F&& f, Args&&... args)
    requires std::is_invocable_v<F, Args...>
    {
        SemaphoreHandle_t semaphore = xSemaphoreCreateBinary();
        
        using archive = std::tuple<
            SemaphoreHandle_t,
            std::decay_t<F>,
            std::conditional_t<std::is_lvalue_reference_v<Args>, Args, std::remove_reference_t<Args>>...
        >;
        archive tuple(semaphore, std::forward<F>(f), std::forward<Args>(args)...);

        TaskFunction_t proxy = +[](void* data)
        {
            archive args = std::move(*reinterpret_cast<archive*>(data));
            
            std::apply(
                [](SemaphoreHandle_t semaphore, std::decay_t<F>& f, auto&&... args){
                    xSemaphoreGive(semaphore);
                    
                    std::invoke(f, std::forward<decltype(args)>(args)...);
                },
                args
            );
        };

        auto ret = xTaskCreate(proxy, name.data(), 4096, &tuple, tskIDLE_PRIORITY, &_handle);
        if (ret != pdPASS)
            throw std::runtime_error("task creating failed");

        xSemaphoreTake(semaphore, portMAX_DELAY);
        vSemaphoreDelete(semaphore);
    }

    void suspend()
    {
        vTaskSuspend(_handle);
    }

    void resume()
    {
        vTaskResume(_handle);
    }

protected:

    TaskHandle_t _handle;
};
    
} // namespace eventide

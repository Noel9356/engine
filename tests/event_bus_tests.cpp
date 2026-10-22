#include <catch2/catch_test_macros.hpp>

#include "engine/kernel/event_bus.hpp"

#include <string>

namespace
{

struct TestEvent
{
    int value = 0;
};

struct OtherEvent
{
    std::string tag;
};

class Listener
{
public:
    void onTestEvent(const TestEvent& event)
    {
        lastValue = event.value;
        ++callCount;
    }

    int lastValue = 0;
    int callCount = 0;
};

} // namespace

TEST_CASE("events reach a member function handler", "[event_bus]")
{
    engine::EventBus bus;
    engine::EventChannel<TestEvent>& channel = bus.channel<TestEvent>();

    Listener listener;
    const auto token = channel.subscribe<Listener, &Listener::onTestEvent>(&listener);

    TestEvent event;
    event.value = 41;
    channel.publish(event);

    REQUIRE(listener.lastValue == 41);
    REQUIRE(listener.callCount == 1);

    REQUIRE(channel.unsubscribe(token));
}

TEST_CASE("handlers fire in subscription order", "[event_bus]")
{
    engine::EventBus bus;
    engine::EventChannel<TestEvent>& channel = bus.channel<TestEvent>();

    std::string order;
    struct Marker
    {
        std::string label;
        std::string* sink = nullptr;
        static void handle(void* userData, const TestEvent&)
        {
            auto* marker = static_cast<Marker*>(userData);
            *marker->sink += marker->label;
        }
    };

    Marker firstMarker{"1", &order};
    Marker secondMarker{"2", &order};
    channel.subscribe(&Marker::handle, &firstMarker);
    channel.subscribe(&Marker::handle, &secondMarker);

    channel.publish(TestEvent{});
    REQUIRE(order == "12");
}

TEST_CASE("unsubscribing inside a handler defers removal", "[event_bus]")
{
    engine::EventBus bus;
    engine::EventChannel<TestEvent>& channel = bus.channel<TestEvent>();

    Listener survivor;
    Listener leaver;

    const auto survivorToken = channel.subscribe<Listener, &Listener::onTestEvent>(&survivor);
    const auto leaverToken = channel.subscribe<Listener, &Listener::onTestEvent>(&leaver);
    (void)survivorToken;

    struct Remover
    {
        engine::EventChannel<TestEvent>* channelPointer = nullptr;
        engine::SubscriptionToken token = engine::invalidSubscriptionToken;
        bool triggered = false;

        static void handle(void* userData, const TestEvent&)
        {
            auto* remover = static_cast<Remover*>(userData);
            remover->triggered = true;
            remover->channelPointer->unsubscribe(remover->token);
        }
    };

    Remover remover{&channel, leaverToken, false};
    const auto removerToken = channel.subscribe(&Remover::handle, &remover);
    (void)removerToken;

    channel.publish(TestEvent{});
    REQUIRE(remover.triggered);
    REQUIRE(leaver.callCount == 1);
    REQUIRE(survivor.callCount == 1);

    // The deferred removal took effect: the second dispatch skips the removed
    // handler without crashing on iteration.
    channel.publish(TestEvent{});
    REQUIRE(survivor.callCount == 2);
    REQUIRE(leaver.callCount == 1);
}

TEST_CASE("RAII subscriptions unsubscribe on destruction", "[event_bus]")
{
    engine::EventBus bus;
    Listener listener;
    {
        engine::EventChannel<TestEvent>& channel = bus.channel<TestEvent>();
        engine::EventSubscription subscription(
            &bus, typeid(TestEvent),
            channel.subscribe<Listener, &Listener::onTestEvent>(&listener));
        channel.publish(TestEvent{});
        REQUIRE(listener.callCount == 1);
    }
    bus.channel<TestEvent>().publish(TestEvent{});
    REQUIRE(listener.callCount == 1); // no dangling call after the guard died
}

TEST_CASE("channels are type isolated and stable", "[event_bus]")
{
    engine::EventBus bus;
    engine::EventChannel<TestEvent>& first = bus.channel<TestEvent>();
    engine::EventChannel<TestEvent>& again = bus.channel<TestEvent>();
    engine::EventChannel<OtherEvent>& other = bus.channel<OtherEvent>();

    REQUIRE(&first == &again);
    REQUIRE(&first != static_cast<void*>(&other));

    OtherEvent otherEvent{"separate"};
    bus.publish(otherEvent); // bus level publish does not disturb TestEvent
}

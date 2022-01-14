#include <shared_mutex>

namespace detail {

// traits:

template<class Mutex>
struct WriteLock
{
    using Type = std::unique_lock<Mutex>;
};

template<class Mutex>
using WriteLockType = typename WriteLock<Mutex>::Type;


template<class Mutex>
struct ReadLock
{
    using Type = std::unique_lock<Mutex>;
};

template<>
struct ReadLock<std::shared_timed_mutex>
{
    using Type = std::shared_lock<std::shared_timed_mutex>;
};

template<>
struct ReadLock<std::shared_mutex>
{
    using Type = std::shared_lock<std::shared_mutex>;
};

template<class Mutex>
using ReadLockType = typename ReadLock<Mutex>::Type;


template<class, class, class, class, class>
class LockingSFINAE;


template<class>
struct isLockingHelper : std::false_type {};

template<class Type, class Mutex, class ReadLock, class WriteLock, class Tag>
struct isLockingHelper<LockingSFINAE<Type, Mutex, ReadLock, WriteLock, Tag>> : std::true_type {};

template<class Type>
struct isLocking : public isLockingHelper<std::decay_t<Type>> {};

template<class Type>
inline constexpr bool isLockingValue = isLocking<Type>::value;


template<class Type>
using IterateType = decltype(*std::cbegin(std::declval<Type>()));

template<class Type, class = void>
struct isIterateLocking : std::false_type {};

template<class Type>
struct isIterateLocking<Type, std::void_t<IterateType<Type>>> : public isLocking<IterateType<Type>> {};

template <class Key, class Value>
struct isIterateLocking<std::pair<const Key, Value>> : isIterateLocking<Value> {};

template<class Type>
inline constexpr bool isIterateLockingValue = isIterateLocking<Type>::value;


// passthrough to element holding locked mutex:

template<typename Type, class Lock>
class Locked
{
public:

    Locked() = delete;

    explicit Locked(const Locked & other) = delete;
    Locked & operator=(const Locked & other) = delete;

    explicit Locked(Locked && other) = delete;
    Locked & operator=(Locked && other) = delete;

    ~Locked() = default;

    template<class Arg>
    auto operator[](Arg && arg) -> decltype(std::declval<Type &>()[std::declval<Arg>()])
    {
        return m_pointer[std::forward<Arg>(arg)];
    }

    Type * operator->()
    {
        return m_pointer;
    }

    // only allow direct dereferencing as lvalue since the lock is released on destruction:
    Type & operator*() && = delete;
    Type & operator*() &
    {
        return *m_pointer;
    }

protected:

    template<typename Mutex>
    explicit Locked(Type * pointer, Mutex & mutex) :
        m_pointer{pointer},
        m_lock{mutex}
    {}

private:

    Type * m_pointer;

    Lock m_lock;

    template<typename OtherType, class Mutex, class ReadLock, class WriteLock, class>
    friend class LockingSFINAE;
};



// container for element together with its mutex:


template<class Type, class Mutex, class ReadLock, class WriteLock, class = void>
class LockingSFINAE
{
    using ReadLocked = Locked<const Type, ReadLock>;

    using WriteLocked = Locked<Type, WriteLock>;

public:

    // capture the default constructor among others:
    template<class...Args>
    LockingSFINAE(Args&&...args) :
        m_element{std::forward<Args>(args)...}
    {}

    // make all 1-argument-constructors but the ones converting from Type/LockingSFINAE explicit:
    template<class Arg, std::enable_if_t<not isLockingValue<Arg>, bool> = true>
    explicit LockingSFINAE(Arg && arg) :
        m_element{std::forward<Arg>(arg)}
    {}

    // capture the standard copy constructor among others:
    template<class Other,
        std::enable_if_t<isLockingValue<Other> && std::is_lvalue_reference_v<Other>, bool> = true>
    LockingSFINAE(Other && other) : // use is_lvalue_reference_v instead of const Other &.
        m_element{*other.getReadLocked().m_pointer}
    {}

    // capture the standard copy assignment among others:
    template<class Other,
        std::enable_if_t<isLockingValue<Other> && std::is_lvalue_reference_v<Other>, bool> = true>
    LockingSFINAE & operator=(Other && other) // use is_lvalue_reference_v instead of const Other &.
    {
        Type element = *other.getReadLocked().m_pointer;
        *this = std::move(element);
        return *this;
    }

    // capture the standard move constructor among others:
    template<class Other,
        std::enable_if_t<isLockingValue<Other> && not std::is_lvalue_reference_v<Other>, bool> = true>
    LockingSFINAE(Other && other) :
        m_element{std::move(*other.getWriteLocked().m_pointer)}
    {}

    // capture the standard move assignment among others:
    template<class Other,
        std::enable_if_t<isLockingValue<Other> && not std::is_lvalue_reference_v<Other>, bool> = true>
    LockingSFINAE & operator=(Other && other)
    {
        Type element = std::move(*other.getWriteLocked().m_pointer);
        *this = std::move(element);
        return *this;
    }

    // conversion from Type:
    LockingSFINAE(const Type & element) :
        m_element{element}
    {}

    LockingSFINAE & operator=(const Type & element)
    {
        *getWriteLocked().m_pointer = element;
        return *this;
    }

    LockingSFINAE(Type && element) :
        m_element{std::move(element)}
    {}

    LockingSFINAE & operator=(Type && element)
    {
        *getWriteLocked().m_pointer = std::move(element);
        return *this;
    }

    // not virtual as polymorphism is unavailable outside (and unused inside) the detail namespace:
    ~LockingSFINAE() = default;

    ReadLocked getReadLocked() const
    {
        return ReadLocked{&m_element, m_mutex};
    }

    WriteLocked getWriteLocked()
    {
        return WriteLocked{&m_element, m_mutex};
    }

private:

    Type m_element;

    mutable Mutex m_mutex{};

    friend class LockingSFINAE<Type, Mutex, ReadLock, WriteLock, void>;
};


template<class Type, class Mutex, class ReadLock, class WriteLock>
class LockingSFINAE<Type, Mutex, ReadLock, WriteLock, std::void_t<IterateType<Type>>> :
    public LockingSFINAE<Type, Mutex, ReadLock, WriteLock, int /* != void: use generic template. */>
{
    using IterateLock = std::conditional_t<isIterateLockingValue<Type>, ReadLock, WriteLock>;

    using IterateLocked = Locked<Type, IterateLock>;

public:

    // capture the default constructor among others:
    template<class...Args>
    LockingSFINAE(Args&&...args) :
        LockingSFINAE<Type, Mutex, ReadLock, WriteLock, int>{std::forward<Args>(args)...}
    {}

    // make all 1-argument-constructors but the ones converting from Type/LockingSFINAE explicit:
    // capture the standard copy and move constructors among others:
    template<class Arg, std::enable_if_t<not isLockingValue<Arg>, bool> = true>
    explicit LockingSFINAE(Arg && arg) :
        LockingSFINAE<Type, Mutex, ReadLock, WriteLock, int>{std::forward<Arg>(arg)}
    {}

    // capture the standard copy and move assignment operators among others:
    template<class Other, std::enable_if_t<isLockingValue<Other>, bool> = true>
    LockingSFINAE & operator=(Other && other)
    {
        LockingSFINAE<Type, Mutex, ReadLock, WriteLock, int>::operator=(std::forward<Other>(other));
        return *this;
    }

    // conversion from Type:
    LockingSFINAE(const Type & element) :
        LockingSFINAE<Type, Mutex, ReadLock, WriteLock, int>{element}
    {}

    LockingSFINAE & operator=(const Type & element)
    {
        LockingSFINAE<Type, Mutex, ReadLock, WriteLock, int>::operator=(element);
        return *this;
    }

    LockingSFINAE(Type && element) :
        LockingSFINAE<Type, Mutex, ReadLock, WriteLock, int>{std::move(element)}
    {}

    LockingSFINAE & operator=(Type && element)
    {
        LockingSFINAE<Type, Mutex, ReadLock, WriteLock, int>::operator=(std::move(element));
        return *this;
    }

    // not virtual as polymorphism is unavailable outside (and unused inside) the detail namespace:
    ~LockingSFINAE() = default;

    IterateLocked getIterateLocked()
    {
        return IterateLocked{&this->m_element, this->m_mutex};
    }

};


} // namespace detail


template<class Type,
    class Mutex = std::shared_timed_mutex,
    class ReadLock = typename detail::ReadLockType<Mutex>,
    class WriteLock = typename detail::WriteLockType<Mutex>>
using Locking = detail::LockingSFINAE<Type, Mutex, ReadLock, WriteLock, void>;


#include <iostream>
#include <chrono>
#include <map>
#include <thread>
#include <vector>

template<class Type,
    class Mutex = std::shared_timed_mutex,
    class ReadLock = typename detail::ReadLockType<Mutex>,
    class WriteLock = typename detail::WriteLockType<Mutex>>
class Dummy
{
public:

    template<class...Args>
    Dummy(Args&&...args) :
        m_element{std::forward<Args>(args)...}
    {}

    template<class Arg>
    explicit Dummy(Arg && arg) :
        m_element{std::forward<Arg>(arg)}
    {}

    Dummy(const Dummy &) = default;
    Dummy & operator=(const Dummy &) = default;
    Dummy(Dummy &&) = default;
    Dummy & operator=(Dummy &&) = default;

    Dummy(const Type & element) :
        m_element{element}
    {}

    Dummy & operator=(const Type & element)
    {
        m_element = element;
        return *this;
    }

    Dummy(Type && element) :
        m_element{std::move(element)}
    {}

    Dummy & operator=(Type && element)
    {
        m_element = std::move(element);
        return *this;
    }

    const Type * getReadLocked() const
    {
        return &m_element;
    }

    Type * getIterateLocked()
    {
        return &m_element;
    }

    Type * getWriteLocked()
    {
        return &m_element;
    }

private:

    Type m_element;

};


//TODO adopt work loads: add/remove some elements, update few/many elements, read few/many elements

// template<class T>
// int read(const T & container)
// {
//     int sum{};
//     for (int unused=0; unused < 20; ++unused)
//     {
//         container.readLocked([](auto && container) {
//             for (auto && element : container)
//             {
//                 volatile int x = 0;
//                 for (int i=0; i<200; ++i)
//                 {
//                     x += i;
//                 }
//                 sum += element.readLocked() + x;
//             }
//         });
//     }
//     return sum;
// }


template<class T>
int read(const T & container)
{
    int sum{};
    for (int unused=0; unused < 20; ++unused)
    {
        auto lockedContainer = container.getReadLocked();
        for (auto && element : *lockedContainer)
        {
            volatile int x = 0;
            for (int i=0; i<200; ++i)
            {
                x += i;
            }
            auto lockedElement = element.getReadLocked();
            sum += *lockedElement + x;
        }
    }
    return sum;
}


template<class T>
void write(T & container)
{
    for (int unused=0; unused < 4; ++unused)
    {
        auto lockedContainer = container.getWriteLocked();
        for (auto && element : *lockedContainer)
        {
            volatile int x = 0;
            for (int i=0; i<500; ++i)
            {
                x += i;
            }
            auto lockedElement = element.getWriteLocked();
            *lockedElement = x;
        }
    }
}


template<class T>
void update(T & container)
{
    for (int unused=0; unused < 4; ++unused)
    {
        auto lockedContainer = container.getIterateLocked();
        for (auto && element : *lockedContainer)
        {
            volatile int x = 0;
            for (int i=0; i<1000; ++i)
            {
                x += i;
            }
            auto lockedElement = element.getWriteLocked();
            *lockedElement = x;
        }
    }
}


template<class T>
void measure(const char * msg, T test)
{
    auto start = std::chrono::high_resolution_clock::now();

    int n = 3000;
    for (auto i=0; i<n; ++i)
    {
        test.getWriteLocked()->push_back(1);
    }

    std::vector<std::thread> threads;
    int sum = 0;

    for (int i=0; i<5; ++i)
    {
        if (detail::isLockingValue<decltype(test)>) {
            threads.emplace_back(std::thread{[&]() -> void { write(test); }});
        } else {
            write(test);
        }
    }

    for (int i=0; i<10; ++i)
    {
        if (detail::isLockingValue<decltype(test)>)  {
            threads.emplace_back(std::thread{[&]() -> void { update(test); }});
        } else {
            update(test);
        }
    }

    for (int i=0; i<20; ++i)
    {
        if (detail::isLockingValue<decltype(test)>)  {
            threads.emplace_back(std::thread{[&]() -> void { sum += read(test); }});
        } else {
            read(test);
        }
    }

    for (auto & thread : threads) { thread.join(); }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << msg << ": " <<
        std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count() / 10 / 100.
        << " ms, sum = " << sum << std::endl;
}


// struct X
// {
//     X() { std::cout<<"X"<<std::endl;}
//     X(const X&) { std::cout<<"cp"<<std::endl;}
//     X(X&&) { std::cout<<"mv"<<std::endl;}
//     X& operator=(const X&) { std::cout<<"c="<<std::endl; return *this; }
//     X& operator=(X&&) { std::cout<<"m="<<std::endl; return *this; }
// };


int main()
{
    Locking<std::map<int, Dummy<int>>, std::mutex> tmp = {};
    std::cout << typeid(tmp.getIterateLocked()).name() << std::endl;

    measure("single threaded, elements = none,         container = none        ",
         Dummy<std::vector<Dummy<int>>>{});

    measure("multi threaded,  elements = none,         container = mutex       ",
         Locking<std::vector<Dummy<int>>, std::mutex>{});

    measure("multi threaded,  elements = none,         container = shared_mutex",
         Locking<std::vector<Dummy<int>>, std::shared_mutex>{});

    measure("multi threaded,  elements = mutex,        container = mutex       ",
         Locking<std::vector<Locking<int, std::mutex>>, std::mutex>{});

    measure("multi threaded,  elements = mutex,        container = shared_mutex",
         Locking<std::vector<Locking<int, std::mutex>>, std::shared_mutex>{});

    measure("multi threaded,  elements = shared_mutex, container = mutex       ",
         Locking<std::vector<Locking<int, std::shared_mutex>>, std::mutex>{});

    measure("multi threaded,  elements = shared_mutex, container = shared_mutex",
         Locking<std::vector<Locking<int, std::shared_mutex>>, std::shared_mutex>{});

    return 0;
}

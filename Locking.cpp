#include <shared_mutex>

namespace detail {

// tags:
    
struct NoLocking {};


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
struct ReadLock<std::shared_mutex> // includes also std::shared_timed_mutex.
{
    using Type = std::shared_lock<std::shared_mutex>;
};

template<class Mutex>
using ReadLockType = typename ReadLock<Mutex>::Type;


namespace adl {
    using std::begin;
    template<class Type>
    using IterateType = decltype(*begin(std::declval<Type>()));
}

using adl::IterateType;


template<class, class = void>
struct isIterable : std::false_type {};

template<class Type>
struct isIterable<Type, std::void_t<IterateType<Type>>> : std::true_type {};

template<class Type>
inline constexpr bool isIterableValue = isIterable<Type>::value;


template<class, class, class, class>
class LockingReadable;

template<class, class, class, class>
class LockingWriteable;

template<class, class, class, class>
class LockingIterable;


template<class>
struct isLockingHelper : std::false_type {};

template<class Type, class Mutex, class ReadLock, class WriteLock>
struct isLockingHelper<LockingReadable<Type, Mutex, ReadLock, WriteLock>> : std::true_type {};

template<class Type, class Mutex, class ReadLock, class WriteLock>
struct isLockingHelper<LockingWriteable<Type, Mutex, ReadLock, WriteLock>> : std::true_type {};

template<class Type, class Mutex, class ReadLock, class WriteLock>
struct isLockingHelper<LockingIterable<Type, Mutex, ReadLock, WriteLock>> : std::true_type {};

template<class Type, class ReadLock, class WriteLock>
struct isLockingHelper<LockingReadable<Type, NoLocking, ReadLock, WriteLock>> : std::false_type {};

template<class Type, class ReadLock, class WriteLock>
struct isLockingHelper<LockingWriteable<Type, NoLocking, ReadLock, WriteLock>> : std::false_type {};

template<class Type, class ReadLock, class WriteLock>
struct isLockingHelper<LockingIterable<Type, NoLocking, ReadLock, WriteLock>> : std::false_type {};

template<class Type>
struct isLocking : public isLockingHelper<std::decay_t<Type>> {};

template<class Type>
inline constexpr bool isLockingValue = isLocking<Type>::value;


template<class Type, class = void>
struct isIterateLocking : std::false_type {};

template<class Type>
struct isIterateLocking<Type, std::void_t<IterateType<Type>>> : isLocking<IterateType<Type>> {};

template <class Key, class Value>
struct isIterateLocking<std::pair<const Key, Value>> : isIterateLocking<Value> {};

template<class Type>
inline constexpr bool isIterateLockingValue = isIterateLocking<Type>::value;


// passthrough to element and scope locking the corresponding mutex:

template<typename Type, class Lock>
class LockedBase
{
public:
    LockedBase() = delete;
    LockedBase(const LockedBase & other) = delete;
    LockedBase & operator=(const LockedBase & other) = delete;
    LockedBase(LockedBase && other) = delete;
    LockedBase & operator=(LockedBase && other) = delete;

    ~LockedBase() = default;

    // ------------------------------------------------------------------------------------------
    
    Type * get()
    {
        return m_pointer;
    }

protected:

    template<typename Mutex>
    LockedBase(Type * pointer, Mutex & mutex) :
        m_pointer{pointer},
        m_lock{mutex}
    {}

private:

    Type * m_pointer;

    Lock m_lock;
};

// ==============================================================================================


template<typename Type, class Lock>
class Locked : public LockedBase<Type, Lock>
{
public:

    Locked() = delete;
    Locked(const Locked & other) = delete;
    Locked & operator=(const Locked & other) = delete;
    Locked(Locked && other) = delete;
    Locked & operator=(Locked && other) = delete;

    ~Locked() = default;
    
    // ------------------------------------------------------------------------------------------

    Type * operator->()
    {
        return this->get();
    }

    // only allow direct dereferencing as lvalue since the lock is released on destruction:
    Type & operator*() && = delete;
    Type & operator*() &
    {
        return *this->get();
    }

protected:

    template<typename Mutex>
    Locked(Type * pointer, Mutex & mutex) : LockedBase<Type, Lock>{pointer, mutex} {}

private:

    template<class, class, class, class>
    friend class LockingReadable;

    template<class, class, class, class>
    friend class LockingWriteable;

    template<class, class, class, class>
    friend class LockingIterable;
};

// ==============================================================================================


template<typename Type, class Lock>
class Locked<Type[], Lock> : public LockedBase<Type, Lock>
{
public:
    Locked() = delete;
    Locked(const Locked & other) = delete;
    Locked & operator=(const Locked & other) = delete;
    Locked(Locked && other) = delete;
    Locked & operator=(Locked && other) = delete;

    ~Locked() = default;

    // ------------------------------------------------------------------------------------------
    
    Type & operator[](size_t idx)
    {
        return this->get()[idx];
    }

protected:

    template<class Mutex>
    Locked(Type * pointer, Mutex & mutex) : LockedBase<Type, Lock>{pointer, mutex} {}

private:

    template<class, class, class, class>
    friend class LockingReadable;

    template<class, class, class, class>
    friend class LockingWriteable;

    template<class, class, class, class>
    friend class LockingIterable;
};

// ==============================================================================================


// container for an element together with its mutex:

template<class Type, class Mutex, class ReadLock, class WriteLock>
class LockingReadable
{
protected:

    using ReadLocked = Locked<const Type, ReadLock>;

public:
    
    // capture the default constructor among others:
    template<class...Args, std::enable_if_t<sizeof...(Args) != 1, bool> = true>
    LockingReadable(Args&&...args) :
        m_element{std::forward<Args>(args)...}
    {}

    // make all 1-argument-constructors but the ones converting from Type/Locking explicit:
    template<class Arg, std::enable_if_t<!isLockingValue<Arg>, bool> = true>
    explicit LockingReadable(Arg && arg) :
        m_element{std::forward<Arg>(arg)}
    {}

    // copy constructors:
    LockingReadable(const Type & element) :
        m_element{element}
    {}

    LockingReadable(const LockingReadable & other) :
        m_element{*other.getReadLocked().get()}
    {}

    template<class OtherMutex, class OtherReadLock, class OtherWriteLock>
    LockingReadable(const LockingReadable<Type, OtherMutex, OtherReadLock, OtherWriteLock> & other) :
        m_element{*other.getReadLocked().get()}
    {}

    // move constructors:
    LockingReadable(Type && element) :
        m_element{std::move(element)}
    {}

    template<class OtherMutex, class OtherReadLock, class OtherWriteLock>
    LockingReadable(LockingWriteable<Type, OtherMutex, OtherReadLock, OtherWriteLock> && other) :
        m_element{std::move(*other.getWriteLocked().get())}
    {}
    
    // ------------------------------------------------------------------------------------------

    // assignments:
    LockingReadable & operator=(const LockingReadable & other) = delete;
    LockingReadable & operator=(LockingReadable && other) = delete;

    // not virtual as polymorphism is unavailable outside (and unused inside) the detail namespace:
    ~LockingReadable() = default;

    // ------------------------------------------------------------------------------------------

    ReadLocked getReadLocked() const
    {
        return ReadLocked{&this->m_element, this->m_mutex};
    }

private:

    Type m_element;

    mutable Mutex m_mutex{};

    friend class LockingWriteable<Type, Mutex, ReadLock, WriteLock>;
    friend class LockingIterable<Type, Mutex, ReadLock, WriteLock>;
};

// ==============================================================================================


template<class Type, class Mutex, class ReadLock, class WriteLock>
class LockingWriteable :
    public LockingReadable<Type, Mutex, ReadLock, WriteLock>
{
    using Base = LockingReadable<Type, Mutex, ReadLock, WriteLock>;

    using WriteLocked = Locked<Type, WriteLock>;

public:

    // make all 1-argument-constructors but the ones converting from Type/Locking explicit:
    template<class Arg,
        std::enable_if_t<
            !isLockingValue<Arg> &&
            !std::is_same_v<std::decay_t<Arg>, std::decay_t<Type>
        >, bool> = true>
    explicit LockingWriteable(Arg && arg) : Base{std::forward<Arg>(arg)}
    {}

    // other constructors including the ones converting from Type/Locking:
    template<class...Args>
    LockingWriteable(Args&&...args) : Base{std::forward<Args>(args)...}
    {}
    
    LockingWriteable() = default;
    LockingWriteable(const LockingWriteable & other) = default;
    
    LockingWriteable(LockingWriteable<Type, Mutex, ReadLock, WriteLock> && other) :
        Base{std::move(*other.getWriteLocked().get())}
    {}

    // not virtual as polymorphism is unavailable outside (and unused inside) the detail namespace:
    ~LockingWriteable() = default;

    // ------------------------------------------------------------------------------------------
    
    // copy assignments:
//     LockingWriteable & operator=(const Type & element) const
//     {
//         *getWriteLocked().get() = element;
//         return *this;
//     }

    LockingWriteable & operator=(const LockingWriteable & other) const
    {
        Type element = *other.getReadLocked().get();
        *getWriteLocked().get() = std::move(element);
        return *this;
    }

    template<class OtherMutex, class OtherReadLock, class OtherWriteLock>
    LockingWriteable & operator=(const LockingReadable<Type, OtherMutex, OtherReadLock, OtherWriteLock> & other) const
    {
        Type element = *other.getReadLocked().get();
        *getWriteLocked().get() = std::move(element);
        return *this = element;
    }

    // move assignments:
//     LockingWriteable & operator=(Type && element) const
//     {
//         *getWriteLocked().get() = std::move(element);
//         return *this;
//     }

    LockingWriteable & operator=(LockingWriteable && other) const
    {
        Type element = std::move(*other.getWriteLocked().get());
        *getWriteLocked().get() = std::move(element);
        return *this;
    }

    template<class OtherMutex, class OtherReadLock, class OtherWriteLock>
    LockingWriteable & operator=(LockingWriteable<Type, OtherMutex, OtherReadLock, OtherWriteLock> && other) const
    {
        Type element = std::move(*other.getWriteLocked().get());
        *getWriteLocked().get() = std::move(element);
        return *this;
    }

    // ------------------------------------------------------------------------------------------

    WriteLocked getWriteLocked() const
    {
        return WriteLocked{const_cast<Type*>(&this->m_element), this->m_mutex};
    }

private:

    friend class LockingIterable<Type, Mutex, ReadLock, WriteLock>;
};

// ==============================================================================================


template<class Type, class Mutex, class ReadLock, class WriteLock>
class LockingIterable :
    public std::conditional_t<std::is_const_v<Type>,
        LockingReadable<Type, Mutex, ReadLock, WriteLock>,
        LockingWriteable<Type, Mutex, ReadLock, WriteLock>>
{
    using Base = std::conditional_t<std::is_const_v<Type>,
        LockingReadable<Type, Mutex, ReadLock, WriteLock>,
        LockingWriteable<Type, Mutex, ReadLock, WriteLock>>;
    
    using IterateLocked = std::conditional_t<isIterateLockingValue<Type> || std::is_const_v<Type>, 
        typename Base::ReadLocked,
        typename LockingWriteable<Type, Mutex, ReadLock, WriteLock>::WriteLocked>;

public:

    // make all 1-argument-constructors but the ones converting from Type/Locking explicit:
    template<class Arg,
        std::enable_if_t<
            !isLockingValue<Arg> &&
            !std::is_same_v<std::decay_t<Arg>, std::decay_t<Type>
        >, bool> = true>
    explicit LockingIterable(Arg && arg) : Base{std::forward<Arg>(arg)}
    {}

    // other constructors including the ones converting from Type/Locking:
    template<class...Args>
    LockingIterable(Args&&...args) : Base{std::forward<Args>(args)...}
    {}

    LockingIterable() = default;
    LockingIterable(const LockingIterable & other) = default;
    LockingIterable & operator=(const LockingIterable & other) = default;
    LockingIterable(LockingIterable && other) = default;
    LockingIterable & operator=(LockingIterable && other) = default;
    
    // not virtual as polymorphism is unavailable outside (and unused inside) the detail namespace:
    ~LockingIterable() = default;

    // assignment operators:
    template<class Other>
    LockingIterable & operator=(Other && other)
    {
        Base::operator=(std::forward<Other>(other));
        return *this;
    }

    // ------------------------------------------------------------------------------------------

    IterateLocked getIterateLocked()
    {
        return IterateLocked{&this->m_element, this->m_mutex};
    }

};

// ==============================================================================================


template<class Type,
    class Mutex = std::shared_timed_mutex,
    class ReadLock = ReadLockType<Mutex>,
    class WriteLock = WriteLockType<Mutex>>
using Locking = std::conditional_t<isIterableValue<Type>,
    LockingIterable<Type, Mutex, ReadLock, WriteLock>,
    std::conditional_t<std::is_const_v<Type>,
        LockingReadable<Type, Mutex, ReadLock, WriteLock>,
        LockingWriteable<Type, Mutex, ReadLock, WriteLock>>>;
    
template<class Type, class Mutex = void, class ReadLock = void, class WriteLock = void>
using DummyLocking = Locking<Type, NoLocking, NoLocking, NoLocking>;

} // namespace detail


using detail::Locking;

using detail::DummyLocking;



#include <iostream>
#include <chrono>
#include <map>
#include <thread>
#include <vector>

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


template <class T>
void type_name(T && t) {
    std::cout << __PRETTY_FUNCTION__ << std::endl;
}


int main()
{
    Locking<const std::vector<Locking<int>>> x{1};
    auto y = x.getIterateLocked();
    auto z = y->at(0).getWriteLocked();
    type_name(z);
    
    Locking<std::map<int, DummyLocking<int>>, std::mutex> tmp = {};
    std::cout << typeid(tmp.getIterateLocked()).name() << std::endl;

    measure("single threaded, elements = none,         container = none        ",
         DummyLocking<std::vector<DummyLocking<int>>>{});

    measure("multi threaded,  elements = none,         container = mutex       ",
         Locking<std::vector<DummyLocking<int>>, std::mutex>{});

    measure("multi threaded,  elements = none,         container = shared_mutex",
         Locking<std::vector<DummyLocking<int>>, std::shared_mutex>{});

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

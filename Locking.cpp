// clang-format -style=WebKit *.{h,cpp} -i
// clang-tidy --extra-arg=-std=c++17 *.{h,cpp}

// ==============================================================================================
// Locked.h
// ==============================================================================================

#include <functional>
#include <mutex>
#include <type_traits>
#include <utility>

namespace Detail {

// passthrough to scalar (SFINAE to distinguish from array), scope locking the corresponding mutex:

template <class Type, class Lock, class = void>
class Locked {
public:
    Locked() = delete;
    Locked(const Locked& other) = delete;
    Locked& operator=(const Locked& other) = delete;
    Locked(Locked&& other) = default;
    Locked& operator=(Locked&& other) = default;
    ~Locked() = default;

    template <class Mutex, class... Args>
    Locked(Type& element, Mutex& mutex, Args&&... args) // mutable
        noexcept(std::is_nothrow_constructible_v<Lock, Mutex&, decltype(args)...>)
        : lock { mutex, std::forward<Args>(args)... }
        , m_element { element }
    {
    }

    // Get the wrapped element without its lock.
    // CAUTION: The lock is released on destruction of this object; for a temporary object,
    // this will be after the current statement. In contrast, the operator*() enforces an lvalue.
    [[nodiscard]] Type& get() const noexcept
    {
        return m_element.get();
    }

    // Only allow direct dereferencing as non-temporary (the lock is released on destruction):
    [[nodiscard]] Type& operator*() const& noexcept
    {
        return get();
    }

    Type& operator*() && = delete;

    [[nodiscard]] Type* operator->() const noexcept
    {
        return &get();
    }

    Lock lock;

private:
    std::reference_wrapper<Type> m_element;
};

// ------------------------------------------------------------------------------------------

// passthrough to array (SFINAE to distinguish from scalar), scope locking the corresponding mutex:

template <class ArrayType, class Lock>
class Locked<ArrayType, Lock, std::enable_if_t<std::is_array_v<ArrayType>, void>> {
    using Type = std::remove_extent_t<ArrayType>;

public:
    Locked() = delete;
    Locked(const Locked& other) = delete;
    Locked& operator=(const Locked& other) = delete;
    Locked(Locked&& other) = default;
    Locked& operator=(Locked&& other) = default;
    ~Locked() = default;

    template <class Mutex, class... Args>
    Locked(Type* element, Mutex& mutex, Args&&... args) // mutable
        noexcept(std::is_nothrow_constructible_v<Lock, Mutex&, decltype(args)...>)
        : lock { mutex, std::forward<Args>(args)... }
        , m_element { element }
    {
    }

    // Get the wrapped element without its lock.
    // ATTENTION: The lock is released on destruction of this object; for a temporary object,
    // this will be after the current statement.
    [[nodiscard]] Type* get() const noexcept
    {
        return m_element;
    }

    [[nodiscard]] Type& operator[](size_t index) const noexcept
    {
        return get()[index];
    }

    Lock lock;

private:
    Type* m_element;
};

} // namespace Detail

// ------------------------------------------------------------------------------------------

using Detail::Locked;

// creator functions can partially deduce template (contrary to constructors or deduction guides):
template <class Lock, class Type, class Mutex, class... Args>
[[nodiscard]] Locked<Type, Lock> makeLocked(Type& element, Mutex& mutex, Args&&... args) // mutable
    noexcept(std::is_nothrow_constructible_v<Locked<Type, Lock>, Type&, Mutex&, decltype(args)...>)
{
    return { element, mutex, std::forward<Args>(args)... };
}

template <template <class> class TLock = std::lock_guard, class Type, class Mutex, class... Args>
[[nodiscard]] Locked<Type, TLock<Mutex>> makeLocked(Type& element, Mutex& mutex, Args&&... args) // mutable
    noexcept(std::is_nothrow_constructible_v<Locked<Type, TLock<Mutex>>, Type&, Mutex&, decltype(args)...>)
{
    return { element, mutex, std::forward<Args>(args)... };
}

// ==============================================================================================
// Locking.h
// ==============================================================================================

//#include <Locked.h"
#include <iterator>
#include <shared_mutex>

template <class, class, class, class>
class Locking;

namespace Detail {

// tags:

struct NoLocking {
    template <class... Args>
    NoLocking(Args...) { }
};

// ------------------------------------------------------------------------------------------

// traits:

template <class Mutex>
struct WriteLock {
    using Type = std::unique_lock<Mutex>;
};

template <class Mutex>
using WriteLockType = typename WriteLock<Mutex>::Type;

// ------------------------------------------------------------------------------------------

template <class Mutex>
struct ReadLock {
    using Type = std::unique_lock<Mutex>;
};

template <>
struct ReadLock<std::shared_mutex> {
    using Type = std::shared_lock<std::shared_mutex>;
};

template <>
struct ReadLock<std::shared_timed_mutex> {
    using Type = std::shared_lock<std::shared_timed_mutex>;
};

template <class Mutex>
using ReadLockType = typename ReadLock<Mutex>::Type;

// ------------------------------------------------------------------------------------------

template <class T>
struct PointerIfUnboundedArray {
    using Type = T;
};

template <class T>
struct PointerIfUnboundedArray<T[]> {
    using Type = T*;
};

template <class Type>
using PointerIfUnboundedArrayType = typename PointerIfUnboundedArray<Type>::Type;

// ------------------------------------------------------------------------------------------

namespace ADL {
    using std::begin;
    template <class Type>
    using IterateType = decltype(*begin(std::declval<Type&>()));
}

using ADL::IterateType;

// ------------------------------------------------------------------------------------------

// conditionals:

template <class, class, class = void>
struct HasNotDeclaredMutexTypeOrIs : public std::true_type {
};

template <class Type, class Mutex>
struct HasNotDeclaredMutexTypeOrIs<Type, Mutex, std::void_t<typename Type::mutex_type>> : public std::is_same<typename Type::mutex_type, Mutex> {
};

template <class Type, class Mutex>
constexpr bool hasNotDeclaredMutexTypeOrIs = HasNotDeclaredMutexTypeOrIs<Type, Mutex>::value;

// ------------------------------------------------------------------------------------------

template <class, class = void>
struct IsIterable : public std::false_type {
};

template <class Type>
struct IsIterable<Type, std::void_t<IterateType<Type>>> : public std::true_type {
};

template <class Type>
constexpr bool isIterable = IsIterable<Type>::value;

// ------------------------------------------------------------------------------------------

template <class>
struct IsDummyLockingHelper : public std::false_type {
};

template <class Type, class ReadLock, class WriteLock>
struct IsDummyLockingHelper<Locking<Type, NoLocking, ReadLock, WriteLock>> : public std::true_type {
};

template <class Type>
struct IsDummyLocking : public IsDummyLockingHelper<std::decay_t<Type>> {
};

template <class Type>
constexpr bool isDummyLocking = IsDummyLocking<Type>::value;

// ------------------------------------------------------------------------------------------

template <class>
struct IsLockingHelper : public std::false_type {
};

template <class Type, class Mutex, class ReadLock, class WriteLock>
struct IsLockingHelper<Locking<Type, Mutex, ReadLock, WriteLock>> : public std::true_type {
};

template <class Type>
struct IsLocking : public std::conjunction<IsLockingHelper<std::decay_t<Type>>, std::negation<IsDummyLocking<Type>>> {
};

template <class Type>
constexpr bool isLocking = IsLocking<Type>::value;

// ------------------------------------------------------------------------------------------

template <class Type, class = void>
struct IsIterateLocking : std::false_type {
};

template <class Type>
struct IsIterateLocking<Type, std::void_t<IterateType<Type>>> : IsLocking<IterateType<Type>> {
};

template <class Key, class Value> // for std::{map,multimap,unordered_map,unordered_multimap}
struct IsIterateLocking<std::pair<const Key, Value>> : IsIterateLocking<Value> {
};

template <class Type>
constexpr bool isIterateLocking = IsIterateLocking<Type>::value;

// ==============================================================================================

template <class, class, class, class>
class LockingIterable;

// container for an element together with its mutex, accessed by getReadLocked and getWriteLocked:
template <class Type, class Mutex, class ReadLock, class WriteLock>
class LockingBase {
    using ReadLocked = Locked<const Type, ReadLock>;

    using WriteLocked = Locked<Type, WriteLock>;

public:
    template <class... Args>
    [[nodiscard]] ReadLocked getReadLocked(Args&&... args) const // mutable
        noexcept(std::is_nothrow_constructible_v<ReadLocked, Type&, Mutex&, decltype(args)...>)
    {
        return { this->m_element, this->m_mutex, std::forward<Args>(args)... };
    }

    // TODO
    /** @brief Get a reference to the container together with a lock on its Mutex.
     * It is intended for modifying the contained elements, while reading (e.g. iterating)
     * the container itself only; cf. https://en.cppreference.com/w/cpp/container#Thread_safety
     * "Different elements in the same container can be modified concurrently by different threads,
     *  except for the elements of std::vector<bool>" (the latter is taken into consideration, too).
     * @param args... forwarded to the constructor of the IterateLock beside the Mutex.
     *
     * @details
     * Internally, it depends how the Mutex is locked, i.e., what is used as IterateLock:
     *   * If the elements of the container are of type Locking<...> (but not DummyLocking<...>),
     *     IterateLock = ReadLock is imposed on its Mutex; typically it is a std::shared_lock<...>.
     *     NB: Contrary to getReadLocked(...) the returned reference is not const.
     *   * If the elements of the container have another type or are DummyLocking<...>, it is the
     *     same as getWriteLocked(...): IterateLock = WriteLock; typically it is a std::unique_lock.
     * Hereby, instead of std::pair<const Key, Value> the Value is used as type of the elements.
     * This is the case for, e.g. std::{map,multimap,unordered_map,unordered_multimap}.
     */
    template <class... Args>
    [[nodiscard]] WriteLocked getWriteLocked(Args&&... args) // mutable
        noexcept(std::is_nothrow_constructible_v<WriteLocked, Type&, Mutex&, decltype(args)...>)
    {
        return { this->m_element, this->m_mutex, std::forward<Args>(args)... };
    }

protected:
    // forward all constructors to create Type element:
    template <class... Args>
    LockingBase(Args&&... args) // mutable
        noexcept(std::is_nothrow_constructible_v<Type, decltype(args)...>)
        : m_element(std::forward<Args>(args)...)
    {
    }

    ~LockingBase() = default;

private:
    PointerIfUnboundedArrayType<Type> m_element; // for Type=T[] use T* instead.

    mutable Mutex m_mutex {};

    friend class LockingIterable<Type, Mutex, ReadLock, WriteLock>;
};

// ------------------------------------------------------------------------------------------

// extension of LockingBase for containers, adding getIterateLocked for modifying its elements;
// all iterables are considered containers, i.e., when begin(container) is dereferencable.
template <class Type, class Mutex, class ReadLock, class WriteLock>
class LockingIterable : public LockingBase<Type, Mutex, ReadLock, WriteLock> {
    using Base = LockingBase<Type, Mutex, ReadLock, WriteLock>;

    using IterateLock = std::conditional_t<isIterateLocking<Type>, ReadLock, WriteLock>;
    // NB: isIterateLocking<std::vector<bool>> == false.

    using IterateLocked = Locked<Type, IterateLock>;

public:
    /** @class Locking
     *  @brief Get a reference to the container together with a lock on its Mutex.
     * It is intended for modifying the contained elements, while reading (e.g. iterating)
     * the container itself only; cf. https://en.cppreference.com/w/cpp/container#Thread_safety
     * "Different elements in the same container can be modified concurrently by different threads,
     *  except for the elements of std::vector<bool>" (the latter is taken into consideration, too).
     *
     * @attention Use getWriteLocked(...) for changing the container itself, e.g. to insert, erase,
     * emplace, extract, push_back, pop_back, resize, clear, merge, splice, reverse, sort, ...
     * @note Often a operator[](...) accesses elements only (and an IterateLock suffices), but in
     * some cases it can change the container (needing a WriteLock), e.g. std::map::operator[](Key).
     *
     * @param args... forwarded to the constructor of the IterateLock beside the Mutex.
     *
     * @details
     * Internally, it depends how the Mutex is locked, i.e., what is used as IterateLock:
     *   * If the elements of the container are of type Locking<...> (but not DummyLocking<...>),
     *     its Mutex is locked by IterateLock = ReadLock; typically it is a std::shared_lock<...>.
     *     NB: Contrary to getReadLocked(...) the returned reference is not const.
     *   * If the elements of the container have another type or are DummyLocking<...>, it has the
     *     same effect as getWriteLocked(...), i.e., the Mutex is locked by IterateLock = WriteLock;
     *     typically it is a std::unique_lock.
     * Hereby, instead of std::pair<const Key, Value> the Value itself is used as type of the
     * elements. This is the case for, e.g. std::{map,multimap,unordered_map,unordered_multimap}.
     */
    template <class... Args>
    [[nodiscard]] IterateLocked getIterateLocked(Args&&... args) // mutable
        noexcept(std::is_nothrow_constructible_v<IterateLocked, Type&, Mutex&, decltype(args)...>)
    {
        return { this->m_element, this->m_mutex, std::forward<Args>(args)... };
    }

protected:
    // forward all constructors:
    using Base::Base;

    ~LockingIterable() = default;
};

} // namespace Detail

// ------------------------------------------------------------------------------------------

/// \extends Detail::LockingIterable<Type, Mutex, ReadLock, WriteLock>
// public interface (constructors and assignments), inherits getReadLocked() and getWriteLocked();
// enables getIterateLocked() iff. the Type is iterable.
template <class Type,
    class Mutex = std::shared_timed_mutex,
    class ReadLock = Detail::ReadLockType<Mutex>,
    class WriteLock = Detail::WriteLockType<Mutex>>
class Locking : public std::conditional_t<Detail::isIterable<Type>,
                    Detail::LockingIterable<Type, Mutex, ReadLock, WriteLock>,
                    Detail::LockingBase<Type, Mutex, ReadLock, WriteLock>> {
    using Base = std::conditional_t<Detail::isIterable<Type>,
        Detail::LockingIterable<Type, Mutex, ReadLock, WriteLock>,
        Detail::LockingBase<Type, Mutex, ReadLock, WriteLock>>;

    static_assert(std::is_same_v<std::remove_cv_t<Type>, Type>,
        "Locking must contain a non-const, non-volatile Type");

    static_assert(std::is_same_v<std::remove_cv_t<Mutex>, Mutex>,
        "Locking must have a non-const, non-volatile Mutex");

    static_assert(std::is_same_v<std::remove_cv_t<ReadLock>, ReadLock>,
        "Locking must have a non-const, non-volatile ReadLock");

    static_assert(std::is_same_v<std::remove_cv_t<WriteLock>, WriteLock>,
        "Locking must have a non-const, non-volatile WriteLock");

    static_assert(Detail::hasNotDeclaredMutexTypeOrIs<ReadLock, Mutex>,
        "Locking's Mutex must be the same as the mutex_type of its ReadLock");

    static_assert(Detail::hasNotDeclaredMutexTypeOrIs<WriteLock, Mutex>,
        "Locking's Mutex must be the same as the mutex_type of its WriteLock");

public:
    // capture the default constructor among others:
    template <class... Args,
        std::enable_if_t<sizeof...(Args) != 1, bool> = true>
    Locking(Args&&... args) // mutable
        noexcept(std::is_nothrow_constructible_v<Base, decltype(args)...>)
        : Base { std::forward<Args>(args)... }
    {
    }

    // make all 1-argument-constructors but the ones converting from Type/Locking<Type,...> explicit:
    template <class Arg,
        std::enable_if_t<!Detail::isLocking<Arg> && !Detail::isDummyLocking<Arg>, bool> = true>
    explicit Locking(Arg&& arg) // mutable
        noexcept(std::is_nothrow_constructible_v<Base, decltype(arg)>)
        : Base { std::forward<Arg>(arg) }
    {
    }

    // forward implicit convertibles to Type (for Type=T[] use T* instead):
    template <class Element,
        std::enable_if_t<std::is_convertible_v<Element, Detail::PointerIfUnboundedArrayType<Type>>, bool> = true>
    Locking(Element&& element) // mutable
        noexcept(std::is_nothrow_constructible_v<Base, decltype(element)>)
        : Base { std::forward<Element>(element) }
    {
    }

    // convert Locking with same Type but maybe different mutex/locks:
    template <class OtherMutex, class OtherReadLock, class OtherWriteLock>
    Locking(Locking<Type, OtherMutex, OtherReadLock, OtherWriteLock> other) // mutable
        noexcept(noexcept(other.getWriteLocked().get())
            && std::is_nothrow_constructible_v<Base, Type&&>)
        : Base { std::move(other.getWriteLocked().get()) }
    {
    }

    // copy constructor:
    Locking(const Locking& other) // mutable
        noexcept(noexcept(other.getReadLocked().get())
            && std::is_nothrow_constructible_v<Base, const Type&>)
        : Base { other.getReadLocked().get() }
    {
    }

    // move constructor:
    Locking(Locking&& other) // mutable
        noexcept(noexcept(other.getWriteLocked().get())
            && std::is_nothrow_constructible_v<Base, Type&&>)
        : Base { std::move(other.getWriteLocked().get()) }
    {
    }

    // destructor:
    virtual ~Locking() = default;

    // assignment for convertibles to Type:
    template <class Element,
        std::enable_if_t<std::is_convertible_v<Element, Type>, bool> = true>
    Locking& operator=(Element&& element) // mutable
        noexcept(noexcept(std::declval<Locking&>().getWriteLocked().get())
            && std::is_nothrow_assignable_v<Type, decltype(element)>)
    {
        this->getWriteLocked().get() = std::forward<Element>(element);
        return *this;
    }

    // copy assignment:
    Locking& operator=(const Locking& other) // mutable
        noexcept(noexcept(other.getReadLocked().get())
            && std::is_nothrow_assignable_v<Locking, Type&&> && std::is_nothrow_copy_assignable_v<Type>)
    {
        Type element = other.getReadLocked().get();
        return *this = std::move(element);
    }

    // move assignment:
    Locking& operator=(Locking&& other) // mutable
        noexcept(noexcept(other.getWriteLocked().get())
            && std::is_nothrow_assignable_v<Locking, Type&&> && std::is_nothrow_move_assignable_v<Type>)

    {
        Type element = std::move(other.getWriteLocked().get());
        return *this = std::move(element);
    }
};

template <class Type, class Mutex, class ReadLock, class WriteLock, class OtherType, class OtherMutex, class OtherReadLock, class OtherWriteLock>
[[nodiscard]] bool operator==(const Locking<Type, Mutex, ReadLock, WriteLock>& lhs, const Locking<OtherType, OtherMutex, OtherReadLock, OtherWriteLock>& rhs) // mutable
    noexcept(noexcept(lhs.getReadLocked().get() == rhs.getReadLocked().get()))
{
    if (&lhs == &rhs) { // sic: prevent locking both!
        return true;
    }
    return lhs.getReadLocked().get() == rhs.getReadLocked().get();
}

// Replacement for the Locking class that disables locking, but provides the same interface;
// usable for easy performance evaluation by changing the declaration only.
// NB: Internally it is recognized that a DummyLocking class is not really a Locking class.
template <class Type, class = void, class = void, class = void>
using DummyLocking = Locking<Type, Detail::NoLocking, Detail::NoLocking, Detail::NoLocking>;

// ==============================================================================================
// test.cpp
// ==============================================================================================

#include <chrono>
#include <iostream>
#include <map>
#include <thread>
#include <vector>

template <class T>
int read(const T& container)
{
    int sum {};
    for (int unused = 0; unused < 20; ++unused) {
        auto lockedContainer = container.getReadLocked();
        for (auto&& element : *lockedContainer) {
            volatile int x = 0;
            for (int i = 0; i < 200; ++i) {
                x += i;
            }
            auto lockedElement = element.getReadLocked();
            sum += *lockedElement + x;
        }
    }
    return sum;
}

template <class T>
void write(T& container)
{
    for (int unused = 0; unused < 4; ++unused) {
        auto lockedContainer = container.getWriteLocked();
        for (auto&& element : *lockedContainer) {
            volatile int x = 0;
            for (int i = 0; i < 500; ++i) {
                x += i;
            }
            auto lockedElement = element.getWriteLocked();
            *lockedElement = x;
        }
    }
}

template <class T>
void update(T& container)
{
    for (int unused = 0; unused < 4; ++unused) {
        auto lockedContainer = container.getIterateLocked();
        for (auto&& element : *lockedContainer) {
            volatile int x = 0;
            for (int i = 0; i < 1000; ++i) {
                x += i;
            }
            auto lockedElement = element.getWriteLocked();
            *lockedElement = x;
        }
    }
}

template <class T>
void measure(const char* msg, T test)
{
    auto start = std::chrono::high_resolution_clock::now();

    int n = 3000;
    for (auto i = 0; i < n; ++i) {
        test.getWriteLocked()->push_back(1);
    }

    std::vector<std::thread> threads;
    int sum = 0;

    for (int i = 0; i < 5; ++i) {
        if (Detail::isLocking<decltype(test)>) {
            threads.emplace_back(std::thread { [&]() -> void { write(test); } });
        } else {
            write(test);
        }
    }

    for (int i = 0; i < 10; ++i) {
        if (Detail::isLocking<decltype(test)>) {
            threads.emplace_back(std::thread { [&]() -> void { update(test); } });
        } else {
            update(test);
        }
    }

    for (int i = 0; i < 20; ++i) {
        if (Detail::isLocking<decltype(test)>) {
            threads.emplace_back(std::thread { [&]() -> void { sum += read(test); } });
        } else {
            read(test);
        }
    }

    for (auto& thread : threads) {
        thread.join();
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << msg << ": " << std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count() / 10 / 100.
              << " ms, sum = " << sum << std::endl;
}

struct X {
    X(int)
    noexcept { std::cout << "Xint" << std::endl; }
    X()
    noexcept { std::cout << "X" << std::endl; }
    X(const X&) { std::cout << "cp" << std::endl; }
    X(X&&) { std::cout << "mv" << std::endl; }
    X& operator=(const X&)
    {
        std::cout << "c=" << std::endl;
        return *this;
    }
    X& operator=(X&&)
    {
        std::cout << "m=" << std::endl;
        return *this;
    }
    ~X() { std::cout << "~X" << std::endl; }
    int x {};
    void operator+(int) { std::cout << "+" << std::endl; }
};
struct Y : X {
    using X::X;
    Y(double) { }
};

template <class T>
auto type_name(T&& t)
{
#if defined(_MSC_VER)
    return __FUNCSIG__;
#else
    return __PRETTY_FUNCTION__;
#endif
}

// not for msvc
//  #include <string_view>
//
//  template <typename T>
//  constexpr auto type_name1()
//  {
//      std::string_view name, prefix, suffix;
//  #ifdef __clang__
//      name = __PRETTY_FUNCTION__;
//      prefix = "auto type_name() [T = ";
//      suffix = "]";
//  #elif defined(__GNUC__)
//      name = __PRETTY_FUNCTION__;
//      prefix = "constexpr auto type_name() [with T = ";
//      suffix = "]";
//  #elif defined(_MSC_VER)
//      name = __FUNCSIG__;
//      prefix = "auto __cdecl type_name<";
//      suffix = ">(void)";
//  #endif
//      name.remove_prefix(prefix.size());
//      name.remove_suffix(suffix.size());
//      return name;
//  }

// TODO: test constructors! noexcept
// TODO: adopt work loads; add/remove some elements, update few/many elements, read few/many elements
// TODO: split tests off
// TODO: namespace overall and for traits
// TODO: modules

int main(int, char**)
{
    X x {};
    Y y { 1. };

    // creator functions:
    int i = 0;
    std::shared_mutex m;
    std::shared_lock { m };
    {
        auto l = makeLocked<std::unique_lock<std::shared_mutex>>(i, m);
        ++*l;
    }
    {
        auto l = makeLocked<std::shared_lock>(i, m);
        std::cout << *l << " pst\n";
    }
    {
        auto l0 = makeLocked(i, m);
        // error: auto l1 = std::move(l0);
        auto l1 = makeLocked<std::shared_lock>(i, m);
        auto l2 = std::move(l1);
    }
    {
        // error: std::cout<<*makeLocked(i, m)<<" pst\n";
        std::cout << makeLocked(i, m).get() << " pst\n";
    }

    // arrays:
    {
        Locking<int[]> is = new int[3];
        auto lis = is.getWriteLocked();
        lis.get()[0] = 23;
        std::cout << lis[0] << " pst\n";
        delete[] lis.get();

        int pis[] = { 5, 7 };
        pis[1] = 3;
        auto l = makeLocked<std::shared_lock>(pis, m);
        l[1] = 5;
        std::cout << l.get()[1] << " pst\n";
    }
    {
        Locking<int[4]> is;
        auto lis = is.getIterateLocked();
        lis.get()[0] = 17;
        std::cout << lis[0] << " pst\n";

        int pis[3];
        auto l = makeLocked(pis, m);
        l[1] = 5;
        std::cout << l.get()[1] << " pst\n";
    }

    Locking<X> x1 {};
    const Locking<X> x2 {};
    auto tmp2 = x2.getReadLocked().get();

    Locking<int> i1, i2;
    bool b1 = i1 == i2;
#if __cplusplus == 202002L
    bool b2 = i1 != i2; // is declared automatically
#endif

    //     X z;
    //     Locking<X> y;
    //     auto l = y.getWriteLocked();
    //     z = std::move(l.get());
    //     l = z;
    //     z = y.getReadLocked();
    //     z = y.getWriteLocked();

    // not for msvc
    //          Locking<std::array<int, 3>> js;
    //          std::cout << type_name(js.getIterateLocked()->operator[](0)) << std::endl;
    //          auto jsl = js.getReadLocked();
    //          for (auto j : *jsl) {
    //              std::cout << j << std::endl;
    //          }
    //
    //      Locking<int, std::mutex, std::lock_guard<std::mutex>, std::lock_guard<std::mutex>> x {};
    //      auto read = x.getReadLocked();

    std::vector<double> testx1 { 1, 2 };
    Locking<std::vector<Locking<double>>> test1 { 1, 2 };
    auto test2 = test1.getIterateLocked();
    auto& w = test2->at(0);
    const Locking<double>& r = test2->at(0);
    auto test3 = w.getWriteLocked(std::defer_lock);
    std::cout << type_name(r.getReadLocked(std::defer_lock)) << std::endl;

    Locking<std::map<int, DummyLocking<int>>, std::mutex> tmp = {};
    std::cout << type_name(tmp.getIterateLocked()) << std::endl;

    measure("single threaded, elements = none,         container = none        ",
        DummyLocking<std::vector<DummyLocking<int>>> {});

    measure("multi threaded,  elements = none,         container = mutex       ",
        Locking<std::vector<DummyLocking<int>>, std::mutex> {});

    measure("multi threaded,  elements = none,         container = shared_mutex",
        Locking<std::vector<DummyLocking<int>>, std::shared_mutex> {});

    measure("multi threaded,  elements = mutex,        container = mutex       ",
        Locking<std::vector<Locking<int, std::mutex>>, std::mutex> {});

    measure("multi threaded,  elements = mutex,        container = shared_mutex",
        Locking<std::vector<Locking<int, std::mutex>>, std::shared_mutex> {});

    measure("multi threaded,  elements = shared_mutex, container = mutex       ",
        Locking<std::vector<Locking<int, std::shared_mutex>>, std::mutex> {});

    measure("multi threaded,  elements = shared_mutex, container = shared_mutex",
        Locking<std::vector<Locking<int, std::shared_mutex>>, std::shared_mutex> {});

    return 0;
}

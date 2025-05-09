//  lock-free single-producer/single-consumer ringbuffer
//  this algorithm is implemented in various projects (linux kernel)
//
//  Copyright (C) 2009-2013 Tim Blechmann
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_LOCKFREE_SPSC_QUEUE_HPP_INCLUDED
#define BOOST_LOCKFREE_SPSC_QUEUE_HPP_INCLUDED

#include <boost/config.hpp>
#ifdef BOOST_HAS_PRAGMA_ONCE
#    pragma once
#endif

#include <algorithm>
#include <memory>
#include <type_traits>

#include <boost/aligned_storage.hpp>
#include <boost/assert.hpp>
#include <boost/core/allocator_access.hpp>
#include <boost/core/span.hpp>
#include <boost/parameter/optional.hpp>
#include <boost/parameter/parameters.hpp>
#include <boost/static_assert.hpp>

#include <boost/lockfree/detail/atomic.hpp>
#include <boost/lockfree/detail/copy_payload.hpp>
#include <boost/lockfree/detail/parameter.hpp>
#include <boost/lockfree/detail/prefix.hpp>
#include <boost/lockfree/detail/uses_optional.hpp>
#include <boost/lockfree/lockfree_forward.hpp>

namespace boost { namespace lockfree {
namespace detail {

template < typename T >
class ringbuffer_base
{
#ifndef BOOST_DOXYGEN_INVOKED
protected:
    typedef std::size_t  size_t;
    static constexpr int padding_size = cacheline_bytes - sizeof( size_t );
    atomic< size_t >     write_index_;
    char                 padding1[ padding_size ]; /* force read_index and write_index to different cache lines */
    atomic< size_t >     read_index_;

protected:
    ringbuffer_base( void ) :
        write_index_( 0 ),
        read_index_( 0 )
    {}

    static size_t next_index( size_t arg, size_t max_size )
    {
        size_t ret = arg + 1;
        while ( BOOST_UNLIKELY( ret >= max_size ) )
            ret -= max_size;
        return ret;
    }

    static size_t read_available( size_t write_index, size_t read_index, size_t max_size )
    {
        if ( write_index >= read_index )
            return write_index - read_index;

        const size_t ret = write_index + max_size - read_index;
        return ret;
    }

    static size_t write_available( size_t write_index, size_t read_index, size_t max_size )
    {
        size_t ret = read_index - write_index - 1;
        if ( write_index >= read_index )
            ret += max_size;
        return ret;
    }

    size_t read_available( size_t max_size ) const
    {
        size_t       write_index = write_index_.load( memory_order_acquire );
        const size_t read_index  = read_index_.load( memory_order_relaxed );
        return read_available( write_index, read_index, max_size );
    }

    size_t write_available( size_t max_size ) const
    {
        size_t       write_index = write_index_.load( memory_order_relaxed );
        const size_t read_index  = read_index_.load( memory_order_acquire );
        return write_available( write_index, read_index, max_size );
    }


    bool push( const T& t, T* buffer, size_t max_size )
    {
        const size_t write_index = write_index_.load( memory_order_relaxed ); // only written from push thread
        const size_t next        = next_index( write_index, max_size );

        if ( next == read_index_.load( memory_order_acquire ) )
            return false;                    /* ringbuffer is full */

        new ( buffer + write_index ) T( t ); // copy-construct

        write_index_.store( next, memory_order_release );

        return true;
    }

    bool push( T&& t, T* buffer, size_t max_size )
    {
        const size_t write_index = write_index_.load( memory_order_relaxed ); // only written from push thread
        const size_t next        = next_index( write_index, max_size );

        if ( next == read_index_.load( memory_order_acquire ) )
            return false;                                         /* ringbuffer is full */

        new ( buffer + write_index ) T( std::forward< T >( t ) ); // move-construct

        write_index_.store( next, memory_order_release );

        return true;
    }

    size_t push( const T* input_buffer, size_t input_count, T* internal_buffer, size_t max_size )
    {
        return push( input_buffer, input_buffer + input_count, internal_buffer, max_size ) - input_buffer;
    }

    template < typename ConstIterator >
    ConstIterator push( ConstIterator begin, ConstIterator end, T* internal_buffer, size_t max_size )
    {
        // FIXME: avoid std::distance

        const size_t write_index = write_index_.load( memory_order_relaxed ); // only written from push thread
        const size_t read_index  = read_index_.load( memory_order_acquire );
        const size_t avail       = write_available( write_index, read_index, max_size );

        if ( avail == 0 )
            return begin;

        size_t input_count = std::distance( begin, end );
        input_count        = ( std::min )( input_count, avail );

        size_t new_write_index = write_index + input_count;

        const ConstIterator last = std::next( begin, input_count );

        if ( write_index + input_count > max_size ) {
            /* copy data in two sections */
            const size_t        count0   = max_size - write_index;
            const ConstIterator midpoint = std::next( begin, count0 );

            std::uninitialized_copy( begin, midpoint, internal_buffer + write_index );
            std::uninitialized_copy( midpoint, last, internal_buffer );
            new_write_index -= max_size;
        } else {
            std::uninitialized_copy( begin, last, internal_buffer + write_index );

            if ( new_write_index == max_size )
                new_write_index = 0;
        }

        write_index_.store( new_write_index, memory_order_release );
        return last;
    }

    template < typename Functor >
    bool consume_one( Functor&& functor, T* buffer, size_t max_size )
    {
        const size_t write_index = write_index_.load( memory_order_acquire );
        const size_t read_index  = read_index_.load( memory_order_relaxed ); // only written from pop thread
        if ( empty( write_index, read_index ) )
            return false;

        T& object_to_consume = buffer[ read_index ];
        functor( std::move( object_to_consume ) );
        object_to_consume.~T();

        size_t next = next_index( read_index, max_size );
        read_index_.store( next, memory_order_release );
        return true;
    }

    template < typename Functor >
    size_t consume_all( Functor&& functor, T* internal_buffer, size_t max_size )
    {
        const size_t write_index = write_index_.load( memory_order_acquire );
        const size_t read_index  = read_index_.load( memory_order_relaxed ); // only written from pop thread

        const size_t avail = read_available( write_index, read_index, max_size );

        if ( avail == 0 )
            return 0;

        const size_t output_count = avail;

        size_t new_read_index = read_index + output_count;

        if ( read_index + output_count > max_size ) {
            /* copy data in two sections */
            const size_t count0 = max_size - read_index;
            const size_t count1 = output_count - count0;

            run_functor_and_delete( internal_buffer + read_index, internal_buffer + max_size, functor );
            run_functor_and_delete( internal_buffer, internal_buffer + count1, functor );

            new_read_index -= max_size;
        } else {
            run_functor_and_delete( internal_buffer + read_index, internal_buffer + read_index + output_count, functor );

            if ( new_read_index == max_size )
                new_read_index = 0;
        }

        read_index_.store( new_read_index, memory_order_release );
        return output_count;
    }

    size_t pop( T* output_buffer, size_t output_count, T* internal_buffer, size_t max_size )
    {
        const size_t write_index = write_index_.load( memory_order_acquire );
        const size_t read_index  = read_index_.load( memory_order_relaxed ); // only written from pop thread

        const size_t avail = read_available( write_index, read_index, max_size );

        if ( avail == 0 )
            return 0;

        output_count = ( std::min )( output_count, avail );

        size_t new_read_index = read_index + output_count;

        if ( read_index + output_count > max_size ) {
            /* copy data in two sections */
            const size_t count0 = max_size - read_index;
            const size_t count1 = output_count - count0;

            move_and_delete( internal_buffer + read_index, internal_buffer + max_size, output_buffer );
            move_and_delete( internal_buffer, internal_buffer + count1, output_buffer + count0 );

            new_read_index -= max_size;
        } else {
            move_and_delete( internal_buffer + read_index, internal_buffer + read_index + output_count, output_buffer );
            if ( new_read_index == max_size )
                new_read_index = 0;
        }

        read_index_.store( new_read_index, memory_order_release );
        return output_count;
    }

    template < typename OutputIterator >
    size_t pop_to_output_iterator( OutputIterator it, T* internal_buffer, size_t max_size )
    {
        const size_t write_index = write_index_.load( memory_order_acquire );
        const size_t read_index  = read_index_.load( memory_order_relaxed ); // only written from pop thread

        const size_t avail = read_available( write_index, read_index, max_size );
        if ( avail == 0 )
            return 0;

        size_t new_read_index = read_index + avail;

        if ( read_index + avail > max_size ) {
            /* copy data in two sections */
            const size_t count0 = max_size - read_index;
            const size_t count1 = avail - count0;

            it = move_and_delete( internal_buffer + read_index, internal_buffer + max_size, it );
            move_and_delete( internal_buffer, internal_buffer + count1, it );

            new_read_index -= max_size;
        } else {
            move_and_delete( internal_buffer + read_index, internal_buffer + read_index + avail, it );
            if ( new_read_index == max_size )
                new_read_index = 0;
        }

        read_index_.store( new_read_index, memory_order_release );
        return avail;
    }

    const T& front( const T* internal_buffer ) const
    {
        const size_t read_index = read_index_.load( memory_order_relaxed ); // only written from pop thread
        return *( internal_buffer + read_index );
    }

    T& front( T* internal_buffer )
    {
        const size_t read_index = read_index_.load( memory_order_relaxed ); // only written from pop thread
        return *( internal_buffer + read_index );
    }
#endif


public:
    /** reset the ringbuffer
     *
     * \note Not thread-safe
     * */
    void reset( void )
    {
        if ( !std::is_trivially_destructible< T >::value ) {
            // make sure to call all destructors!
            consume_all( []( const T& ) {} );
        } else {
            write_index_.store( 0, memory_order_relaxed );
            read_index_.store( 0, memory_order_release );
        }
    }

    /** Check if the ringbuffer is empty
     *
     * \return true, if the ringbuffer is empty, false otherwise
     * \note Due to the concurrent nature of the ringbuffer the result may be inaccurate.
     * */
    bool empty( void )
    {
        return empty( write_index_.load( memory_order_relaxed ), read_index_.load( memory_order_relaxed ) );
    }

    /**
     * \return true, if implementation is lock-free.
     *
     * */
    bool is_lock_free( void ) const
    {
        return write_index_.is_lock_free() && read_index_.is_lock_free();
    }

private:
    bool empty( size_t write_index, size_t read_index )
    {
        return write_index == read_index;
    }

    template < class OutputIterator >
    OutputIterator move_and_delete( T* first, T* last, OutputIterator out )
    {
        if ( std::is_trivially_destructible< T >::value ) {
            return std::copy( first, last, out ); // will use memcpy if possible
        } else {
            for ( ; first != last; ++first, ++out ) {
                *out = std::move( *first );
                first->~T();
            }
            return out;
        }
    }

    template < class Functor >
    void run_functor_and_delete( T* first, T* last, Functor&& functor )
    {
        for ( ; first != last; ++first ) {
            functor( std::move( *first ) );
            first->~T();
        }
    }
};

template < typename T, std::size_t MaxSize >
class compile_time_sized_ringbuffer : public ringbuffer_base< T >
{
    typedef std::size_t          size_type;
    static constexpr std::size_t max_size = MaxSize + 1;

    typedef
        typename boost::aligned_storage< max_size * sizeof( T ), boost::alignment_of< T >::value >::type storage_type;

    storage_type storage_;

    T* data()
    {
        return static_cast< T* >( storage_.address() );
    }

    const T* data() const
    {
        return static_cast< const T* >( storage_.address() );
    }

protected:
    size_type max_number_of_elements() const
    {
        return max_size;
    }

    ~compile_time_sized_ringbuffer( void )
    {
        // destroy all remaining items
        consume_all( []( const T& ) {} );
    }

public:
    bool push( const T& t )
    {
        return ringbuffer_base< T >::push( t, data(), max_size );
    }

    bool push( T&& t )
    {
        return ringbuffer_base< T >::push( std::forward< T >( t ), data(), max_size );
    }

    template < typename Functor >
    bool consume_one( Functor&& f )
    {
        return ringbuffer_base< T >::consume_one( f, data(), max_size );
    }

    template < typename Functor >
    size_type consume_all( Functor&& f )
    {
        return ringbuffer_base< T >::consume_all( f, data(), max_size );
    }

    size_type push( T const* t, size_type size )
    {
        return ringbuffer_base< T >::push( t, size, data(), max_size );
    }

    template < size_type size >
    size_type push( T const ( &t )[ size ] )
    {
        return push( t, size );
    }

    template < typename ConstIterator >
    ConstIterator push( ConstIterator begin, ConstIterator end )
    {
        return ringbuffer_base< T >::push( begin, end, data(), max_size );
    }

    size_type pop( T* ret, size_type size )
    {
        return ringbuffer_base< T >::pop( ret, size, data(), max_size );
    }

    template < typename OutputIterator >
    size_type pop_to_output_iterator( OutputIterator it )
    {
        return ringbuffer_base< T >::pop_to_output_iterator( it, data(), max_size );
    }

    const T& front( void ) const
    {
        return ringbuffer_base< T >::front( data() );
    }

    T& front( void )
    {
        return ringbuffer_base< T >::front( data() );
    }
};

template < typename T, typename Alloc >
class runtime_sized_ringbuffer : public ringbuffer_base< T >, private Alloc
{
    typedef std::size_t                        size_type;
    size_type                                  max_elements_;
    typedef std::allocator_traits< Alloc >     allocator_traits;
    typedef typename allocator_traits::pointer pointer;
    pointer                                    array_;

protected:
    size_type max_number_of_elements() const
    {
        return max_elements_;
    }

public:
    explicit runtime_sized_ringbuffer( size_type max_elements ) :
        max_elements_( max_elements + 1 )
    {
        Alloc& alloc = *this;
        array_       = allocator_traits::allocate( alloc, max_elements_ );
    }

    template < typename U >
    runtime_sized_ringbuffer( typename boost::allocator_rebind< Alloc, U >::type const& alloc, size_type max_elements ) :
        Alloc( alloc ),
        max_elements_( max_elements + 1 )
    {
        Alloc& allocator = *this;
        array_           = allocator_traits::allocate( allocator, max_elements_ );
    }

    runtime_sized_ringbuffer( Alloc const& alloc, size_type max_elements ) :
        Alloc( alloc ),
        max_elements_( max_elements + 1 )
    {
        Alloc& allocator = *this;
        array_           = allocator_traits::allocate( allocator, max_elements_ );
    }

    ~runtime_sized_ringbuffer( void )
    {
        // destroy all remaining items
        consume_all( []( const T& ) {} );

        Alloc& allocator = *this;
        allocator_traits::deallocate( allocator, array_, max_elements_ );
    }

    bool push( const T& t )
    {
        return ringbuffer_base< T >::push( t, &*array_, max_elements_ );
    }

    bool push( T&& t )
    {
        return ringbuffer_base< T >::push( std::forward< T >( t ), &*array_, max_elements_ );
    }

    template < typename Functor >
    bool consume_one( Functor&& f )
    {
        return ringbuffer_base< T >::consume_one( f, &*array_, max_elements_ );
    }

    template < typename Functor >
    size_type consume_all( Functor&& f )
    {
        return ringbuffer_base< T >::consume_all( f, &*array_, max_elements_ );
    }

    size_type push( T const* t, size_type size )
    {
        return ringbuffer_base< T >::push( t, size, &*array_, max_elements_ );
    }

    template < size_type size >
    size_type push( T const ( &t )[ size ] )
    {
        return push( t, size );
    }

    template < typename ConstIterator >
    ConstIterator push( ConstIterator begin, ConstIterator end )
    {
        return ringbuffer_base< T >::push( begin, end, &*array_, max_elements_ );
    }

    size_type pop( T* ret, size_type size )
    {
        return ringbuffer_base< T >::pop( ret, size, &*array_, max_elements_ );
    }

    template < typename OutputIterator >
    size_type pop_to_output_iterator( OutputIterator it )
    {
        return ringbuffer_base< T >::pop_to_output_iterator( it, &*array_, max_elements_ );
    }

    const T& front( void ) const
    {
        return ringbuffer_base< T >::front( &*array_ );
    }

    T& front( void )
    {
        return ringbuffer_base< T >::front( &*array_ );
    }
};

typedef parameter::parameters< boost::parameter::optional< tag::capacity >, boost::parameter::optional< tag::allocator > >
    ringbuffer_signature;

template < typename T, typename... Options >
struct make_ringbuffer
{
    typedef typename ringbuffer_signature::bind< Options... >::type bound_args;

    typedef extract_capacity< bound_args > extract_capacity_t;

    static constexpr bool   runtime_sized = !extract_capacity_t::has_capacity;
    static constexpr size_t capacity      = extract_capacity_t::capacity;

    typedef extract_allocator< bound_args, T > extract_allocator_t;
    static constexpr bool                      has_allocator = extract_allocator_t::has_allocator;
    typedef typename extract_allocator_t::type allocator;

    static constexpr bool signature_is_valid = runtime_sized ? true : !has_allocator;
    BOOST_STATIC_ASSERT( signature_is_valid );

    typedef std::conditional_t< runtime_sized,
                                runtime_sized_ringbuffer< T, allocator >,
                                compile_time_sized_ringbuffer< T, capacity > >
        ringbuffer_type;
};


} /* namespace detail */


/** The spsc_queue class provides a single-writer/single-reader fifo queue, pushing and popping is wait-free.
 *
 *  \b Policies:
 *  - \c boost::lockfree::capacity<>, optional <br>
 *    If this template argument is passed to the options, the size of the ringbuffer is set at compile-time.
 *
 *  - \c boost::lockfree::allocator<>, defaults to \c boost::lockfree::allocator<std::allocator<T>> <br>
 *    Specifies the allocator that is used to allocate the ringbuffer. This option is only valid, if the ringbuffer is
 * configured to be sized at run-time
 *
 *  \b Requirements:
 *  - T must have a default constructor
 *  - T must be copyable or movable
 * */
template < typename T, typename... Options >
#if !defined( BOOST_NO_CXX20_HDR_CONCEPTS )
    requires( std::is_default_constructible_v< T >, std::is_move_assignable_v< T > || std::is_copy_assignable_v< T > )
#endif
class spsc_queue : public detail::make_ringbuffer< T, Options... >::ringbuffer_type
{
private:
#ifndef BOOST_DOXYGEN_INVOKED
    typedef typename detail::make_ringbuffer< T, Options... >::ringbuffer_type base_type;
    static constexpr bool runtime_sized = detail::make_ringbuffer< T, Options... >::runtime_sized;
    typedef typename detail::make_ringbuffer< T, Options... >::allocator allocator_arg;

    struct implementation_defined
    {
        typedef allocator_arg allocator;
        typedef std::size_t   size_type;
    };
#endif

public:
    typedef T                                          value_type;
    typedef typename implementation_defined::allocator allocator;
    typedef typename implementation_defined::size_type size_type;

    /** Constructs a spsc_queue
     *
     *  \pre spsc_queue must be configured to be sized at compile-time
     */
    spsc_queue( void )
#if !defined( BOOST_NO_CXX20_HDR_CONCEPTS )
        requires( !runtime_sized )
#endif
    {
        // Don't use BOOST_STATIC_ASSERT() here since it will be evaluated when compiling
        // this function and this function may be compiled even when it isn't being used.
        BOOST_ASSERT( !runtime_sized );
    }

    /** Constructs a spsc_queue with a custom allocator
     *
     *  \pre spsc_queue must be configured to be sized at compile-time
     *
     *  \note This is just for API compatibility: an allocator isn't actually needed
     */
    template < typename U, typename Enabler = std::enable_if< !runtime_sized > >
    explicit spsc_queue( typename boost::allocator_rebind< allocator, U >::type const& )
    {}

    /** Constructs a spsc_queue with a custom allocator
     *
     *  \pre spsc_queue must be configured to be sized at compile-time
     *
     *  \note This is just for API compatibility: an allocator isn't actually needed
     */
    template < typename Enabler = std::enable_if< !runtime_sized > >
    explicit spsc_queue( allocator const& )
    {}

    /** Constructs a spsc_queue for element_count elements
     *
     *  \pre spsc_queue must be configured to be sized at run-time
     */
    template < typename Enabler = std::enable_if< runtime_sized > >
    explicit spsc_queue( size_type element_count ) :
        base_type( element_count )
    {}

    /** Constructs a spsc_queue for element_count elements with a custom allocator
     *
     *  \pre spsc_queue must be configured to be sized at run-time
     */
    template < typename U, typename Enabler = std::enable_if< runtime_sized > >
    spsc_queue( size_type element_count, typename boost::allocator_rebind< allocator, U >::type const& alloc ) :
        base_type( alloc, element_count )
    {}

    /** Constructs a spsc_queue for element_count elements with a custom allocator
     *
     *  \pre spsc_queue must be configured to be sized at run-time
     */
    template < typename Enabler = std::enable_if< runtime_sized > >
    spsc_queue( size_type element_count, allocator_arg const& alloc ) :
        base_type( alloc, element_count )
    {}

    spsc_queue( const spsc_queue& )            = delete;
    spsc_queue& operator=( const spsc_queue& ) = delete;
    spsc_queue( spsc_queue&& )                 = delete;
    spsc_queue& operator=( spsc_queue&& )      = delete;

    /** Pushes object t to the ringbuffer.
     *
     * \pre only one thread is allowed to push data to the spsc_queue
     * \post object will be pushed to the spsc_queue, unless it is full.
     * \return true, if the push operation is successful.
     *
     * \note Thread-safe and wait-free
     * */
    bool push( const T& t )
    {
        return base_type::push( t );
    }

    /// \copydoc boost::lockfree::spsc_queue::push(const T& t)
    bool push( T&& t )
    {
        return base_type::push( std::forward< T >( t ) );
    }

    /** Pops one object from ringbuffer.
     *
     * \pre only one thread is allowed to pop data from the spsc_queue
     * \post if ringbuffer is not empty, object will be discarded.
     * \return true, if the pop operation is successful, false if ringbuffer was empty.
     *
     * \note Thread-safe and wait-free
     */
    bool pop()
    {
        return consume_one( []( const T& ) {} );
    }

    /** Pops one object from ringbuffer.
     *
     * \pre only one thread is allowed to pop data from the spsc_queue
     * \post if ringbuffer is not empty, object will be copied to ret.
     * \return true, if the pop operation is successful, false if ringbuffer was empty.
     *
     * \note Thread-safe and wait-free
     */
    template < typename U, typename Enabler = std::enable_if< std::is_convertible< T, U >::value > >
    bool pop( U& ret )
    {
        return consume_one( [ & ]( T&& t ) {
            ret = std::forward< T >( t );
        } );
    }

#if !defined( BOOST_NO_CXX17_HDR_OPTIONAL ) || defined( BOOST_DOXYGEN_INVOKED )
    /** Pops object from spsc_queue, returning a std::optional<>
     *
     * \returns `std::optional` with value if successful, `std::nullopt` if spsc_queue is empty.
     *
     * \note Thread-safe and non-blocking
     *
     * */
    std::optional< T > pop( uses_optional_t )
    {
        T to_dequeue;
        if ( pop( to_dequeue ) )
            return to_dequeue;
        else
            return std::nullopt;
    }

    /** Pops object from spsc_queue, returning a std::optional<>
     *
     * \pre type T must be convertible to U
     * \returns `std::optional` with value if successful, `std::nullopt` if spsc_queue is empty.
     *
     * \note Thread-safe and non-blocking
     *
     * */
    template < typename U >
    std::optional< U > pop( uses_optional_t )
    {
        U to_dequeue;
        if ( pop( to_dequeue ) )
            return to_dequeue;
        else
            return std::nullopt;
    }
#endif

    /** Pushes as many objects from the array t as there is space.
     *
     * \pre only one thread is allowed to push data to the spsc_queue
     * \return number of pushed items
     *
     * \note Thread-safe and wait-free
     */
    size_type push( T const* t, size_type size )
    {
        return base_type::push( t, size );
    }

    /** Pushes as many objects from the array t as there is space available.
     *
     * \pre only one thread is allowed to push data to the spsc_queue
     * \return number of pushed items
     *
     * \note Thread-safe and wait-free
     */
    template < size_type size >
    size_type push( T const ( &t )[ size ] )
    {
        return push( t, size );
    }

    /** Pushes as many objects from the span t as there is space available.
     *
     * \pre only one thread is allowed to push data to the spsc_queue
     * \return number of pushed items
     *
     * \note Thread-safe and wait-free
     */
    template < std::size_t Extent >
    size_type push( boost::span< const T, Extent > t )
    {
        return push( t.data(), t.size() );
    }

    /** Pushes as many objects from the range [begin, end) as there is space .
     *
     * \pre only one thread is allowed to push data to the spsc_queue
     * \return iterator to the first element, which has not been pushed
     *
     * \note Thread-safe and wait-free
     */
    template < typename ConstIterator >
    ConstIterator push( ConstIterator begin, ConstIterator end )
    {
        return base_type::push( begin, end );
    }

    /** Pops a maximum of size objects from ringbuffer.
     *
     * \pre only one thread is allowed to pop data from the spsc_queue
     * \return number of popped items
     *
     * \note Thread-safe and wait-free
     * */
    size_type pop( T* ret, size_type size )
    {
        return base_type::pop( ret, size );
    }

    /** Pops a maximum of size objects from spsc_queue.
     *
     * \pre only one thread is allowed to pop data from the spsc_queue
     * \return number of popped items
     *
     * \note Thread-safe and wait-free
     * */
    template < size_type size >
    size_type pop( T ( &ret )[ size ] )
    {
        return pop( ret, size );
    }

    /** Pops objects to the output iterator it
     *
     * \pre only one thread is allowed to pop data from the spsc_queue
     * \return number of popped items
     *
     * \note Thread-safe and wait-free
     * */
    template < typename OutputIterator >
    typename std::enable_if< !std::is_convertible< T, OutputIterator >::value, size_type >::type pop( OutputIterator it )
    {
        return base_type::pop_to_output_iterator( it );
    }

    /** consumes one element via a functor
     *
     *  pops one element from the queue and applies the functor on this object
     *
     * \returns true, if one element was consumed
     *
     * \note Thread-safe and non-blocking, if functor is thread-safe and non-blocking
     * */
    template < typename Functor >
    bool consume_one( Functor&& f )
    {
        return base_type::consume_one( f );
    }

    /** consumes all elements via a functor
     *
     * sequentially pops all elements from the queue and applies the functor on each object
     *
     * \returns number of elements that are consumed
     *
     * \note Thread-safe and non-blocking, if functor is thread-safe and non-blocking
     * */
    template < typename Functor >
    size_type consume_all( Functor&& f )
    {
        return base_type::consume_all( f );
    }

    /** get number of elements that are available for read
     *
     * \return number of available elements that can be popped from the spsc_queue
     *
     * \note Thread-safe and wait-free, should only be called from the consumer thread
     * */
    size_type read_available() const
    {
        return base_type::read_available( base_type::max_number_of_elements() );
    }

    /** get write space to write elements
     *
     * \return number of elements that can be pushed to the spsc_queue
     *
     * \note Thread-safe and wait-free, should only be called from the producer thread
     * */
    size_type write_available() const
    {
        return base_type::write_available( base_type::max_number_of_elements() );
    }

    /** get reference to element in the front of the queue
     *
     * Availability of front element can be checked using read_available().
     *
     * \pre only a consuming thread is allowed to check front element
     * \pre read_available() > 0. If ringbuffer is empty, it's undefined behaviour to invoke this method.
     * \return reference to the first element in the queue
     *
     * \note Thread-safe and wait-free
     */
    const T& front() const
    {
        BOOST_ASSERT( read_available() > 0 );
        return base_type::front();
    }

    /// \copydoc boost::lockfree::spsc_queue::front() const
    T& front()
    {
        BOOST_ASSERT( read_available() > 0 );
        return base_type::front();
    }

    /** reset the ringbuffer
     *
     * \note Not thread-safe
     * */
    void reset( void )
    {
        if ( !std::is_trivially_destructible< T >::value ) {
            // make sure to call all destructors!
            consume_all( []( const T& ) {} );
        } else {
            base_type::write_index_.store( 0, memory_order_relaxed );
            base_type::read_index_.store( 0, memory_order_release );
        }
    }
};

}} // namespace boost::lockfree


#endif /* BOOST_LOCKFREE_SPSC_QUEUE_HPP_INCLUDED */

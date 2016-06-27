#include <cstddef>
#include <new>

/**
 * @brief   Define non-existing function to catch the usage of new.
 */
void you_tried_to_use_new(void);

/**
 * @brief   Define non-existing function to catch the usage of delete.
 */
void you_tried_to_use_delete(void);

/**
 * @brief   Redefinition of new to stop the usage of it.
 */
void *operator new(std::size_t size) throw(std::bad_alloc)
{
    (void) size;
    you_tried_to_use_new();
    return NULL;
}

/**
 * @brief   Redefinition of new to stop the usage of it.
 */
void *operator new(std::size_t size, std::nothrow_t const &) throw()
{
    (void) size;
    you_tried_to_use_new();
    return NULL;
}

/**
 * @brief   Redefinition of new to stop the usage of it.
 */
void *operator new [](std::size_t size) throw (std::bad_alloc)
{
    (void) size;
    you_tried_to_use_new();
    return NULL;
}

/**
 * @brief   Redefinition of new to stop the usage of it.
 */
void *operator new [] (std::size_t size, std::nothrow_t const &) throw ()
{
    (void) size;
    you_tried_to_use_new();
    return NULL;
}

/**
 * @brief   Redefinition of delete to stop the usage of it.
 */
void operator delete(void *p) throw(std::bad_alloc)
{
    (void)p;
    you_tried_to_use_delete();
}

/**
 * @brief   Redefinition of delete to stop the usage of it.
 */
void operator delete(void *p, std::nothrow_t const &) throw()
{
    (void)p;
    you_tried_to_use_delete();
}

/**
 * @brief   Redefinition of delete to stop the usage of it.
 */
void operator delete(void *p, std::size_t size) throw(std::bad_alloc)
{
    (void)p;
    (void)size;
    you_tried_to_use_delete();
}

/**
 * @brief   Redefinition of delete to stop the usage of it.
 */
void operator delete(void *p, std::size_t size, std::nothrow_t const &) throw()
{
    (void)p;
    (void)size;
    you_tried_to_use_delete();
}

#include <CircularBuffer/CircularBuffer.h>
#include <Util/Util.h>

CircularBuffer::CircularBuffer(uint8_t *contents, uint8_t size)
    : _contents(contents), _size(size), _size_mask(size - 1)
{

}

void CircularBuffer::write(uint8_t data)
{
	_contents[_head++ & _size_mask] = data;
}

uint8_t CircularBuffer::read()
{
	return _contents[_tail++ & _size_mask];
}

uint8_t CircularBuffer::used()
{
	return (uint8_t)(_head - _tail);
}

uint8_t CircularBuffer::free()
{
	return _size - (uint8_t)(_head - _tail);
}

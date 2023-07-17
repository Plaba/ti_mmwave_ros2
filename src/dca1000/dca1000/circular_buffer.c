#include <Python.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/*
	To compile for debugging:
		gcc circ_buff.c -o circ_buff && ./circ_buff && rm circ_buff
		This would: - Compile the c file 		(gcc circ_buff.c -o circ_buff)
					- Run the compiled file 	(./circ_buff)
					- Delete the compiled file 	(rm circ_buff)
	To compile for running:
		cc -fPIC -shared -o circ_buff.so circ_buff.c
*/

/*
	This function changes the value of a binary array passed in by reference.
	If the put idx passes the next frame boundary, the pop_array puts a 1 in the correct
	location and updates the get idx to be frame boundary.
*/

int16_t find_pops(int64_t  old_put_idx,
			   int64_t  new_put_idx,
			   int64_t  frame_size)
{
	int32_t old_frame_idx = old_put_idx / frame_size;
	int32_t new_frame_idx = new_put_idx / frame_size;
	return(old_frame_idx == new_frame_idx) ? -1 : old_frame_idx;
}

void add_zeros(int64_t num_zeros,
			   int16_t* buffer,
			   int64_t buffer_len,
			   int64_t* put_idx,
			   int64_t frame_size,
			   int16_t* pop_frame_idx)
{
	int32_t new_put_idx = *put_idx;
	int32_t to_end_of_frame = frame_size - (*put_idx % frame_size); // number of cell to the end of current frame
	if (num_zeros < to_end_of_frame) { // if does not reach the end of the frame
		memset(buffer + *put_idx, 0, num_zeros * sizeof(buffer[0])); // set zeros accordingly
		new_put_idx += num_zeros; // move pointer
	}
	else {	// if overflow
		memset(buffer + *put_idx, 0, to_end_of_frame * sizeof(buffer[0])); // set all unfinished cell of frame to 0
		num_zeros -= to_end_of_frame; // substract those 0 filled
		new_put_idx += to_end_of_frame; // move to start of the next frame
		if (new_put_idx >= buffer_len) new_put_idx -= buffer_len; // loop if necessary

		num_zeros %= frame_size; // the rest of the needed zeros (skip 0-filled frames)
		memset(buffer + new_put_idx, 0, num_zeros * sizeof(buffer[0])); // fill zeros to destination idx
		new_put_idx += num_zeros; // move pointer
	}
	*pop_frame_idx = find_pops(*put_idx, new_put_idx, frame_size);
	*put_idx = new_put_idx;
}

void add_msg(int16_t* msg,
			 int16_t msg_len,
			 int16_t* buffer,
			 int64_t buffer_len,
			 int64_t* put_idx,
			 int64_t frame_size,
			 int16_t* pop_frame_idx)
{
	int32_t new_put_idx = *put_idx;
	if (*put_idx + msg_len <= buffer_len) //did not loop around to beginning
	{
		memcpy(buffer + *put_idx, msg, msg_len*sizeof(msg[0]));
		new_put_idx = *put_idx + msg_len; //new location of put idx
		if (new_put_idx >= buffer_len) new_put_idx -= buffer_len;
	}
	else // did loop around to beginning
	{
		memcpy(buffer + *put_idx, msg, (buffer_len - *put_idx)*sizeof(msg[0]));
		memcpy(buffer, msg + (buffer_len - *put_idx), (msg_len - buffer_len + *put_idx)*sizeof(msg[0]));
		new_put_idx = msg_len - buffer_len + *put_idx;
	}

	*pop_frame_idx = find_pops(*put_idx, new_put_idx, frame_size);
	*put_idx = new_put_idx;
}

void pad_and_add_msg(int64_t seq_c,
        int64_t seq_n,
        int16_t* msg,
        int16_t msg_len,
        int16_t* buffer,
        int64_t buffer_len,
        int64_t* put_idx,
        int64_t frame_size,
        int16_t* pop_frame_idx){
    //determine if zeros needed
    int64_t num_zeros = (seq_n - seq_c - 1) * 728;
    //printf(stderr, "INFO: current sequence number %ld, received %ld\n", seq_c, seq_n);
    if(num_zeros > 0){
        // fprintf(stderr, "WARN: Padding %ld zeros\n", num_zeros);
        add_zeros(num_zeros, buffer, buffer_len, put_idx, frame_size, pop_frame_idx);
    }

    add_msg(msg, msg_len, buffer, buffer_len, put_idx, frame_size, pop_frame_idx);
}

static PyObject*
circular_buffer_add_zeros(PyObject* self, PyObject* args) {

  PyObject *py_num_zeros, *py_buffer, *py_buffer_len, *py_put_idx, *py_frame_size, *py_pop_array;

  if (!PyArg_ParseTuple(args, "OOOOOO",
                        &py_num_zeros, 
                        &py_buffer,
                        &py_buffer_len,
                        &py_put_idx,
                        &py_frame_size,  
                        &py_pop_array)) {
    return NULL;
  }

  int64_t num_zeros = PyLong_AsLongLong(py_num_zeros);

  Py_buffer buffer_view;
  if (PyObject_GetBuffer(py_buffer, &buffer_view, PyBUF_ANY_CONTIGUOUS) == -1) {
    return NULL;
  }

  int16_t* buffer = (int16_t*)buffer_view.buf;
  int64_t buffer_len = buffer_view.len / sizeof(int16_t);

  int64_t put_idx = PyLong_AsLongLong(py_put_idx);
  int64_t frame_size = PyLong_AsLongLong(py_frame_size);
  int16_t pop_array = PyLong_AsLong(py_pop_array);

  add_zeros(num_zeros, buffer, buffer_len, &put_idx, frame_size, &pop_array);

  PyBuffer_Release(&buffer_view);

  Py_RETURN_NONE;  
}

static PyObject*
circular_buffer_add_msg(PyObject* self, PyObject* args) {

  PyObject *py_msg, *py_msg_len, *py_buffer, *py_buffer_len, *py_put_idx, *py_frame_size, *py_pop_array;

  if (!PyArg_ParseTuple(args, "OOOOOOO",
                        &py_msg,
                        &py_msg_len,
                        &py_buffer,
                        &py_buffer_len,
                        &py_put_idx,
                        &py_frame_size,
                        &py_pop_array)) {
    return NULL;
  }

  Py_buffer msg_view;
  if (PyObject_GetBuffer(py_msg, &msg_view, PyBUF_ANY_CONTIGUOUS) == -1) {
    return NULL;
  }

  int16_t* msg = (int16_t*)msg_view.buf;
  int msg_len = PyLong_AsLongLong(py_msg_len);

  Py_buffer buffer_view;
  if (PyObject_GetBuffer(py_buffer, &buffer_view, PyBUF_ANY_CONTIGUOUS) == -1) {
    PyBuffer_Release(&msg_view);
    return NULL;
  }

  int16_t* buffer = (int16_t*)buffer_view.buf;
  int64_t buffer_len = PyLong_AsLongLong(py_buffer_len);

  int64_t put_idx = PyLong_AsLongLong(py_put_idx);
  int64_t frame_size = PyLong_AsLongLong(py_frame_size);
  int16_t pop_array = PyLong_AsLongLong(py_pop_array);

  add_msg(msg, msg_len, buffer, buffer_len, &put_idx, frame_size, &pop_array);

  PyBuffer_Release(&msg_view);
  PyBuffer_Release(&buffer_view);

  Py_RETURN_NONE;
}

static PyObject*
circular_buffer_pad_and_add_msg(PyObject* self, PyObject* args) {

  PyObject *py_seq_c, *py_seq_n, *py_msg, *py_msg_len, *py_buffer, *py_buffer_len, *py_put_idx, *py_frame_size, *py_pop_array;

  if (!PyArg_ParseTuple(args, "OOOOOOOOO",
                        &py_seq_c,
                        &py_seq_n,
                        &py_msg,
                        &py_msg_len,
                        &py_buffer,
                        &py_buffer_len,
                        &py_put_idx,
                        &py_frame_size,
                        &py_pop_array)) {
    return NULL; 
  }

  int64_t seq_c = PyLong_AsLongLong(py_seq_c);
  int64_t seq_n = PyLong_AsLongLong(py_seq_n);

  Py_buffer msg_view;
  if (PyObject_GetBuffer(py_msg, &msg_view, PyBUF_ANY_CONTIGUOUS) == -1) {
    return NULL;
  }

  int16_t* msg = (int16_t*)msg_view.buf;
  int msg_len = PyLong_AsLongLong(py_msg_len);

  Py_buffer buffer_view;
  if (PyObject_GetBuffer(py_buffer, &buffer_view, PyBUF_ANY_CONTIGUOUS) == -1) {
    PyBuffer_Release(&msg_view);
    return NULL;
  }

  int16_t* buffer = (int16_t*)buffer_view.buf;
  int64_t buffer_len = PyLong_AsLongLong(py_buffer_len);  

  int64_t put_idx = PyLong_AsLongLong(py_put_idx);
  int64_t frame_size = PyLong_AsLongLong(py_frame_size);
  int16_t pop_array = PyLong_AsLongLong(py_pop_array);

  pad_and_add_msg(seq_c, seq_n, msg, msg_len, buffer, buffer_len, &put_idx, frame_size, &pop_array);

  PyBuffer_Release(&msg_view);
  PyBuffer_Release(&buffer_view);

  Py_RETURN_NONE;
}

static PyMethodDef circular_buffer_methods[] = {
  {"add_zeros", circular_buffer_add_zeros, METH_FASTCALL, "Add zeros to buffer"},
  {"add_msg", circular_buffer_add_msg,  METH_FASTCALL, "Add message to buffer"},
  {"pad_and_add_msg", circular_buffer_pad_and_add_msg,  METH_FASTCALL, "Pad and add message to buffer"},
  {NULL, NULL, 0, NULL}
};

static struct PyModuleDef circular_buffer_module = {
    PyModuleDef_HEAD_INIT,
    "circular_buffer",
    "Circular buffer C extension",
    -1,
    circular_buffer_methods
};

PyMODINIT_FUNC PyInit_circular_buffer(void) {
    return PyModule_Create(&circular_buffer_module);
}

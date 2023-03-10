/*
 * \file
 *
 * \brief WINC1500 Exosite Example.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include "debug_conf.h"
#include "exosite_example.h"
#include "io1_board.h"
#include "tick_counter.h"

#define EXOSITE_EXAMPLE_HTTP_CONTENT_TYPE		"application/x-www-form-urlencoded; charset=utf-8"
#define EXOSITE_EXAMPLE_HTTP_CIK_EXT_HEADER		"X-Exosite-CIK:%s\r\n"
#define EXOSITE_EXAMPLE_HTTP_ACTIVATE_URL		"http://atmel.m2.exosite.com/provision/activate"
#define EXOSITE_EXAMPLE_HTTP_READ_AND_WRITE_URL "http://atmel.m2.exosite.com/onep:v1/stack/alias?led"

/** Instance of Timer module. */
struct sw_timer_module swt_module_inst;

/** Instance of HTTP client module. */
struct http_client_module http_client_module_inst;

struct http_entity g_http_entity = {0,};

/************************************************************************/
/* declare inner function                                               */
/************************************************************************/
static bool			_exosite_example_configure_http_client(exosite_http_cb cb);
static void			_exosite_example_configure_timer(void);

struct http_entity *_exosite_example_http_set_default_entity(void);
const char*			_exosite_example_http_get_contents_type(void *priv_data);
int					_exosite_example_http_get_contents_length(void *priv_data);
int					_exosite_example_http_read(void *priv_data, char *buffer, uint32_t size, uint32_t written);
void				_exosite_example_http_close(void *priv_data);

bool	exosite_example_init(exosite_http_cb cb)
{
	DEBUG(DEBUG_CONF_EXOSITE"exosite_example_init"DEBUG_EOL);
	/* Initialize the Timer. */
	_exosite_example_configure_timer();
	/* Initialize the HTTP client service. */
	if( !_exosite_example_configure_http_client(cb) )
	{
		DEBUG(DEBUG_CONF_EXOSITE"Error : exosite_example_init"DEBUG_EOL);
		return false;
	}
	return true;
}

bool	exosite_example_activiate(char * activate_data)
{
	bool ret = false;
	int err_code = 0;
	DEBUG(DEBUG_CONF_EXOSITE"exosite_example_activiate"DEBUG_EOL);
	
	struct http_entity * entity = _exosite_example_http_set_default_entity();
	entity->priv_data = (void*)activate_data;
	
	err_code = http_client_send_request(&http_client_module_inst, EXOSITE_EXAMPLE_HTTP_ACTIVATE_URL,HTTP_METHOD_POST,entity, NULL);
	if( err_code == 0)
	{
		ret = true;	
		tick_counter_pending_timer();
	}
	else
	{
		DEBUG(DEBUG_CONF_EXOSITE"Error : exosite_example_activiate code %d"DEBUG_EOL, err_code);
	}
	
	return ret;
}

bool	exosite_example_read_and_write(char * write_data, char * cik)
{
	bool ret = false;
	int err_code = 0;
	char ext_header[100] = {0,};
	struct http_entity * entity = _exosite_example_http_set_default_entity();
	
	DEBUG(DEBUG_CONF_EXOSITE"exosite_example_read_and_write"DEBUG_EOL);
	
	entity->priv_data = (void*)write_data;	
	sprintf(ext_header,EXOSITE_EXAMPLE_HTTP_CIK_EXT_HEADER, cik);	
	
	err_code = http_client_send_request(&http_client_module_inst, EXOSITE_EXAMPLE_HTTP_READ_AND_WRITE_URL,HTTP_METHOD_POST,entity, ext_header);
	
	if( err_code == 0)
	{
		ret = true;		
		tick_counter_pending_timer();
	}
	else
	{
		DEBUG(DEBUG_CONF_EXOSITE"Error : exosite_example_read_and_write code %d"DEBUG_EOL, err_code);
	}
	return ret;
}

/**
 * \brief Configure Timer module.
 */
static void _exosite_example_configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);

	sw_timer_init(&swt_module_inst, &swt_conf);
	sw_timer_enable(&swt_module_inst);
}

/**
 * \brief Configure HTTP client module.
 */
static bool _exosite_example_configure_http_client(exosite_http_cb cb)
{
	struct http_client_config httpc_conf;
	int ret;

	http_client_get_config_defaults(&httpc_conf);

	httpc_conf.recv_buffer_size = 256;
	httpc_conf.send_buffer_size = 1024;
	httpc_conf.timer_inst = &swt_module_inst;
	/* ipinfo.io send json format data if only client is a curl. */
	httpc_conf.user_agent = "curl/7.10.6";

	ret = http_client_init(&http_client_module_inst, &httpc_conf);
	if (ret < 0) {
		DEBUG(DEBUG_CONF_EXOSITE"HTTP client initialization has failed(%s)"DEBUG_EOL, strerror(ret));
		return false;
	}

	http_client_register_callback(&http_client_module_inst, cb);
	return true;
}

struct http_entity * _exosite_example_http_set_default_entity()
{
	memset(&g_http_entity, 0x00, sizeof(struct http_entity));
	g_http_entity.close = _exosite_example_http_close;
	g_http_entity.is_chunked = 0;
	g_http_entity.priv_data = NULL;
	g_http_entity.read = _exosite_example_http_read;
	g_http_entity.get_contents_length = _exosite_example_http_get_contents_length;
	g_http_entity.get_contents_type = _exosite_example_http_get_contents_type;
	
	return &g_http_entity;
}

const char* _exosite_example_http_get_contents_type(void *priv_data)
{
	return (const char*)EXOSITE_EXAMPLE_HTTP_CONTENT_TYPE;
}

int _exosite_example_http_get_contents_length(void *priv_data)
{
	return strlen( (char*)priv_data);
}

int _exosite_example_http_read(void *priv_data, char *buffer, uint32_t size, uint32_t written)
{
	int32_t length = 0;
	
	if(priv_data)
	{
		length = strlen( (char*)priv_data);
		memcpy(buffer,(char*)priv_data, length);
	}
	
	return length;
}

void _exosite_example_http_close(void *priv_data)
{
}
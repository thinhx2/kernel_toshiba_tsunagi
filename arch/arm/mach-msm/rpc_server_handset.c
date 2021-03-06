/*
* Certain software is contributed or developed by TOSHIBA CORPORATION.
* Copyright (C) 2010 TOSHIBA CORPORATION All rights reserved.
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by FSF, and
* may be copied, distributed, and modified under those terms.
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* This code is based on rpc_server_handset.c.
* The original copyright and notice are described below.
*/
/* arch/arm/mach-msm/rpc_server_handset.c
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/switch.h>

#include <asm/mach-types.h>

#include <mach/msm_rpcrouter.h>
#include <mach/board.h>
#include <mach/rpc_server_handset.h>

#define DRIVER_NAME	"msm-handset"

#define HS_SERVER_PROG 0x30000062
#define HS_SERVER_VERS 0x0

#define HS_RPC_PROG 0x30000062

#define HS_RPC_VERS 0x0

#define HS_SUBSCRIBE_SRVC_PROC 0x03
#define HS_REPORT_EVNT_PROC    0x05
#define HS_EVENT_CB_PROC	1
#define HS_EVENT_DATA_VER	1

#define RPC_KEYPAD_NULL_PROC 0
#define RPC_KEYPAD_USB_STATE_CODE_PROC 1
#define RPC_KEYPAD_PASS_KEY_CODE_PROC 2
#define RPC_KEYPAD_SET_PWR_KEY_STATE_PROC 3

#define HS_PWR_K		0x6F	/* Power key */
#define HS_END_K		0x51	/* End key or Power key */
#define HS_STEREO_HEADSET_K	0x82
#define HS_HEADSET_SWITCH_K	0x84

#define HS_SEND_K		0x50	/* Send key */
#define HS_BACK_K		0xC0	/* Back key */
#define HS_MENU_K		0x5B	/* Menu key */
#define HS_HOME_K		0xC1	/* Home key */
#define HS_REMOCON_K		0xE0	/* Remocon key */
#define HS_CAMERA_K             0xA2    /* Camera key */

#define HS_REL_K		0xFF	/* key release */
#define HS_TSUNAGI_K		0x6E	/* Tsunagi Key  */
#define CHG_ST_NTFY_CODE        0xE0    /* battery abnormal charge event */

#define HS_KEY_CHG_ST_NONE         0x155
#define HS_KEY_CHG_ST_OVP          0x156
#define HS_KEY_CHG_ST_OVC          0x157
#define HS_KEY_CHG_ST_OVD          0x158
#define HS_KEY_CHG_ST_EXP          0x159

#define USB_CABLE_DET           0xD0

#define KEY(hs_key, input_key) ((hs_key << 24) | input_key)

enum hs_event {
	HS_EVNT_EXT_PWR = 0,	/* External Power status        */
	HS_EVNT_HSD,		/* Headset Detection            */
	HS_EVNT_HSTD,		/* Headset Type Detection       */
	HS_EVNT_HSSD,		/* Headset Switch Detection     */
	HS_EVNT_KPD,
	HS_EVNT_FLIP,		/* Flip / Clamshell status (open/close) */
	HS_EVNT_CHARGER,	/* Battery is being charged or not */
	HS_EVNT_ENV,		/* Events from runtime environment like DEM */
	HS_EVNT_REM,		/* Events received from HS counterpart on a
				remote processor*/
	HS_EVNT_DIAG,		/* Diag Events  */
	HS_EVNT_LAST,		 /* Should always be the last event type */
	HS_EVNT_MAX		/* Force enum to be an 32-bit number */
};

enum hs_src_state {
	HS_SRC_STATE_UNKWN = 0,
	HS_SRC_STATE_LO,
	HS_SRC_STATE_HI,
};

struct hs_event_data {
	uint32_t	ver;		/* Version number */
	enum hs_event	event_type;     /* Event Type	*/
	enum hs_event	enum_disc;     /* discriminator */
	uint32_t	data_length;	/* length of the next field */
	enum hs_src_state	data;    /* Pointer to data */
	uint32_t	data_size;	/* Elements to be processed in data */
};

enum hs_return_value {
	HS_EKPDLOCKED     = -2,	/* Operation failed because keypad is locked */
	HS_ENOTSUPPORTED  = -1,	/* Functionality not supported */
	HS_FALSE          =  0, /* Inquired condition is not true */
	HS_FAILURE        =  0, /* Requested operation was not successful */
	HS_TRUE           =  1, /* Inquired condition is true */
	HS_SUCCESS        =  1, /* Requested operation was successful */
	HS_MAX_RETURN     =  0x7FFFFFFF/* Force enum to be a 32 bit number */
};

struct hs_key_data {
	uint32_t ver;        /* Version number to track sturcture changes */
	uint32_t code;       /* which key? */
	uint32_t parm;       /* key status. Up/down or pressed/released */
};

enum hs_subs_srvc {
	HS_SUBS_SEND_CMD = 0, /* Subscribe to send commands to HS */
	HS_SUBS_RCV_EVNT,     /* Subscribe to receive Events from HS */
	HS_SUBS_SRVC_MAX
};

enum hs_subs_req {
	HS_SUBS_REGISTER,    /* Subscribe   */
	HS_SUBS_CANCEL,      /* Unsubscribe */
	HS_SUB_STATUS_MAX
};

enum hs_event_class {
	HS_EVNT_CLASS_ALL = 0, /* All HS events */
	HS_EVNT_CLASS_LAST,    /* Should always be the last class type   */
	HS_EVNT_CLASS_MAX
};

enum hs_cmd_class {
	HS_CMD_CLASS_LCD = 0, /* Send LCD related commands              */
	HS_CMD_CLASS_KPD,     /* Send KPD related commands              */
	HS_CMD_CLASS_LAST,    /* Should always be the last class type   */
	HS_CMD_CLASS_MAX
};


enum battery_abnormal_events {
        K_CHG_ST_NONE = 0,
        K_CHG_ST_OVP,
        K_CHG_ST_OVC,
        K_CHG_ST_OVD,
        K_CHG_ST_EXP
};
enum {
	USB_NOTINITIALIZE = 0,
	USB_CLIENTINITIALIZE,
	USB_HOSTINITIALIZE,
};


enum cable_det_param {
	K_CABLE_WAKEUP = 0,             /* aARM Wake Up */
	K_CABLE_USB_OTG_INIT_PHY = 1,   /*USB initialization demand*/
	K_CABLE_USB_CONNECTED = 2,		/*USB Cable connected*/
	K_CABLE_USB_DISCONNECT = 3,     /*The notice of USB cable cutting*/
	K_CABLE_USB_CONNECT_CLIENT = 4, /*The notice of USB client cable connection*/
	K_CABLE_USB_CONNECT_HOST = 5    /*The notice of USB host cable connection*/
        /* And mode ... */
};

/*
 * Receive events or send command
 */
union hs_subs_class {
	enum hs_event_class	evnt;
	enum hs_cmd_class	cmd;
};

struct hs_subs {
	uint32_t                ver;
	enum hs_subs_srvc	srvc;  /* commands or events */
	enum hs_subs_req	req;   /* subscribe or unsubscribe  */
	uint32_t		host_os;
	enum hs_subs_req	disc;  /* discriminator    */
	union hs_subs_class      id;
};

struct hs_event_cb_recv {
	uint32_t cb_id;
	uint32_t hs_key_data_ptr;
	struct hs_key_data key;
};

static const uint32_t hs_key_map[] = {
	KEY(HS_PWR_K, KEY_POWER),
	KEY(HS_END_K, KEY_END),
	KEY(HS_STEREO_HEADSET_K, SW_HEADPHONE_INSERT),
	KEY(HS_HEADSET_SWITCH_K, KEY_MEDIA),
	KEY(HS_SEND_K, KEY_SEND),
	KEY(HS_BACK_K, KEY_BACK),
	KEY(HS_MENU_K, KEY_MENU),
	KEY(HS_HOME_K, KEY_HOME),
    KEY(HS_CAMERA_K, KEY_CAMERA),
    KEY(HS_KEY_CHG_ST_NONE, KEY_CHG_ST_NONE),
    KEY(HS_KEY_CHG_ST_OVP, KEY_CHG_ST_OVP),
    KEY(HS_KEY_CHG_ST_OVC, KEY_CHG_ST_OVC),
    KEY(HS_KEY_CHG_ST_OVD, KEY_CHG_ST_OVD),
    KEY(HS_KEY_CHG_ST_EXP, KEY_CHG_ST_EXP),

	0
};

enum {
	NO_DEVICE	= 0,
	MSM_HEADSET	= 1,
};

struct msm_handset {
	struct input_dev *ipdev;
	struct switch_dev sdev;
};

static struct msm_rpc_client *rpc_client;
static struct msm_handset *hs;

static int hs_find_key(uint32_t hscode)
{
	printk("%s\n", __func__);
	int i, key;

	key = KEY(hscode, 0);

	for (i = 0; hs_key_map[i] != 0; i++) {
		if ((hs_key_map[i] & 0xff000000) == key)
			return hs_key_map[i] & 0x00ffffff;
	}
	return -1;
}

static void
report_headset_switch(struct input_dev *dev, int key, int value)
{
	printk("%s\n", __func__);
	struct msm_handset *hs = input_get_drvdata(dev);

	input_report_switch(dev, key, value);
	switch_set_state(&hs->sdev, value);
}

extern void notify_cable_status(int status);
/*
 * tuple format: (key_code, key_param)
 *
 * old-architecture:
 * key-press = (key_code, 0)
 * key-release = (0xff, key_code)
 *
 * new-architecutre:
 * key-press = (key_code, 0)
 * key-release = (key_code, 0xff)
 */
static void report_hs_key(uint32_t key_code, uint32_t key_parm)
{
	printk("%s\n", __func__);
	int key, temp_key_code;
	printk("<1>KeyCode = 0x%x   KeyPara = 0x%x\n", key_code, key_parm);
    if (key_code == CHG_ST_NTFY_CODE)
    {
            printk("got CHG_ST_NTFY_CODE notification\n");
            switch(key_parm)
            {
                    case K_CHG_ST_NONE:
                            key = KEY_CHG_ST_NONE;
                            break;
                    case K_CHG_ST_OVP:
                            key = KEY_CHG_ST_OVP;
                            break;
                    case K_CHG_ST_OVC:
                            key = KEY_CHG_ST_OVC;
                            break;
                    case K_CHG_ST_OVD:
                            key = KEY_CHG_ST_OVD;
                            break;
                    case K_CHG_ST_EXP:
                            key = KEY_CHG_ST_EXP;
                            break;
            }
            printk(" key_code = %d, key_parm = %d\n", key_code, key_parm);
            input_report_key(hs->ipdev, key, (key_code != HS_REL_K));
		//return;		
    }

    if (key_code == USB_CABLE_DET) {
	    switch(key_parm) {
	    case K_CABLE_WAKEUP:
	             printk(KERN_INFO "%s: cable_det debug. call wakeup (%d)\n", __func__, key_parm);
	    /* call wakeup function */
	
	             break;
	    case K_CABLE_USB_OTG_INIT_PHY:
	             printk(KERN_INFO "%s: cable_det debug. USB utilization demand (%d)\n", __func__, key_parm);
				//msm_usb_phy_reg_init();
			     break;
			     
	    case K_CABLE_USB_CONNECTED:
	             printk(KERN_INFO "%s: cable_det debug. USB connected (%d)\n", __func__, key_parm);
	             notify_cable_status(1);
				//msm_usb_phy_reg_init();
			     break;
			     
	    case K_CABLE_USB_DISCONNECT:
	             printk(KERN_INFO "%s: cable_det debug. USB cable cutting (%d) \n",__func__,key_parm);
	             notify_cable_status(0);
		#if 1
		            //msm_otg_disable_irq();
		#endif
			     break;
		case K_CABLE_USB_CONNECT_CLIENT:
			     printk(KERN_INFO "%s: cable_det debug. USB connect client (%d) \n",__func__,key_parm);
				//msm_otg_enable_irq();
			     break;
	    case K_CABLE_USB_CONNECT_HOST:
				//msm_otg_enable_irq();
			     printk(KERN_INFO "%s: cable_det debug. USB connect host (%d) \n",__func__,key_parm);
			     break;
	
	    default:
	        /* error !! */
	             break;
	      }
        return;
    }
   
    if (key_code == HS_TSUNAGI_K) {
		key = key_parm;
		switch (key) {
		case HS_PWR_K:
			printk("Power Key pressed!\n");
			input_report_key(hs->ipdev, KEY_END, 1);
			input_report_key(hs->ipdev, KEY_END, 0);
		break;
		}
		input_sync(hs->ipdev);
		return;
     }
  

	if (key_code == HS_REL_K)
		key = hs_find_key(key_parm);
	else
		key = hs_find_key(key_code);

	temp_key_code = key_code;

	if (key_parm == HS_REL_K)
		key_code = key_parm;

	switch (key) {
		case KEY_POWER:
		case KEY_END:
		case KEY_MEDIA:
		case KEY_SEND:
		case KEY_BACK:
		case KEY_MENU:
		case KEY_HOME:
			input_report_key(hs->ipdev, key, (key_code != HS_REL_K));
			break;
	        case KEY_CAMERA:
	                input_report_key(hs->ipdev, key, (key_code != HS_REL_K));
	                input_report_key(hs->ipdev, key, HS_REL_K);
	                break;
		case SW_HEADPHONE_INSERT:
			report_headset_switch(hs->ipdev, key, (key_code != HS_REL_K));
			break;
		case -1:
			printk(KERN_ERR "%s: No mapping for remote handset event %d\n",
					 __func__, temp_key_code);
			return;
	}
	input_sync(hs->ipdev);
}

static int handle_hs_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len)
{
	printk("%s CODE: 0x%x\n", __func__, req->procedure);
	struct rpc_keypad_pass_key_code_args {
		uint32_t key_code;
		uint32_t key_parm;
	};

	switch (req->procedure) {
	case RPC_KEYPAD_NULL_PROC:
		return 0;

	case RPC_KEYPAD_USB_STATE_CODE_PROC: {
		struct rpc_keypad_pass_key_code_args *args;

		args = (struct rpc_keypad_pass_key_code_args *)(req + 1);
		args->key_code = be32_to_cpu(args->key_code);
		args->key_parm = be32_to_cpu(args->key_parm);

		report_hs_key(args->key_code, args->key_parm);

		return 0;		
	}

	case RPC_KEYPAD_PASS_KEY_CODE_PROC: {
// We could handle here Power Up/Down from Power Key on Tsunagi, not neded at the moment
/*		struct rpc_keypad_pass_key_code_args *args;

		args = (struct rpc_keypad_pass_key_code_args *)(req + 1);
		args->key_code = be32_to_cpu(args->key_code);
		args->key_parm = be32_to_cpu(args->key_parm);

		report_hs_key(args->key_code, args->key_parm);
*/
		return 0;
	}

	case RPC_KEYPAD_SET_PWR_KEY_STATE_PROC:
		/* This RPC function must be available for the ARM9
		 * to function properly.  This function is redundant
		 * when RPC_KEYPAD_PASS_KEY_CODE_PROC is handled. So
		 * input_report_key is not needed.
		 */
		return 0;
	default:
		return -ENODEV;
	}
}

static struct msm_rpc_server hs_rpc_server = {
	.prog		= HS_SERVER_PROG,
	.vers		= HS_SERVER_VERS,
	.rpc_call	= handle_hs_rpc_call,
};

static int process_subs_srvc_callback(struct hs_event_cb_recv *recv)
{
	printk("%s\n", __func__);
	if (!recv)
		return -ENODATA;

	report_hs_key(be32_to_cpu(recv->key.code), be32_to_cpu(recv->key.parm));

	return 0;
}

static void process_hs_rpc_request(uint32_t proc, void *data)
{
	printk("%s\n", __func__);
	if (proc == HS_EVENT_CB_PROC)
		process_subs_srvc_callback(data);
	else
		pr_err("%s: unknown rpc proc %d\n", __func__, proc);
}

static int hs_rpc_report_event_arg(struct msm_rpc_client *client,
					void *buffer, void *data)
{
	printk("%s\n", __func__);
	struct hs_event_rpc_req {
		uint32_t hs_event_data_ptr;
		struct hs_event_data data;
	};

	struct hs_event_rpc_req *req = buffer;

	req->hs_event_data_ptr	= cpu_to_be32(0x1);
	req->data.ver		= cpu_to_be32(HS_EVENT_DATA_VER);
	req->data.event_type	= cpu_to_be32(HS_EVNT_HSD);
	req->data.enum_disc	= cpu_to_be32(HS_EVNT_HSD);
	req->data.data_length	= cpu_to_be32(0x1);
	req->data.data		= cpu_to_be32(*(enum hs_src_state *)data);
	req->data.data_size	= cpu_to_be32(sizeof(enum hs_src_state));

	return sizeof(*req);
}

static int hs_rpc_report_event_res(struct msm_rpc_client *client,
					void *buffer, void *data)
{
	printk("%s\n", __func__);
	enum hs_return_value result;

	result = be32_to_cpu(*(enum hs_return_value *)buffer);
	pr_debug("%s: request completed: 0x%x\n", __func__, result);

	if (result == HS_SUCCESS)
		return 0;

	return 1;
}

void report_headset_status(bool connected)
{
	printk("%s\n", __func__);
	int rc = -1;
	enum hs_src_state status;

	if (connected == true)
		status = HS_SRC_STATE_HI;
	else
		status = HS_SRC_STATE_LO;

	rc = msm_rpc_client_req(rpc_client, HS_REPORT_EVNT_PROC,
				hs_rpc_report_event_arg, &status,
				hs_rpc_report_event_res, NULL, -1);

	if (rc)
		pr_err("%s: couldn't send rpc client request\n", __func__);
}
EXPORT_SYMBOL(report_headset_status);

static int hs_rpc_register_subs_arg(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	printk("%s\n", __func__);
	struct hs_subs_rpc_req {
		uint32_t hs_subs_ptr;
		struct hs_subs hs_subs;
		uint32_t hs_cb_id;
		uint32_t hs_handle_ptr;
		uint32_t hs_handle_data;
	};

	struct hs_subs_rpc_req *req = buffer;

	req->hs_subs_ptr	= cpu_to_be32(0x1);
	req->hs_subs.ver	= cpu_to_be32(0x1);
	req->hs_subs.srvc	= cpu_to_be32(HS_SUBS_RCV_EVNT);
	req->hs_subs.req	= cpu_to_be32(HS_SUBS_REGISTER);
	req->hs_subs.host_os	= cpu_to_be32(0x4); /* linux */
	req->hs_subs.disc	= cpu_to_be32(HS_SUBS_RCV_EVNT);
	req->hs_subs.id.evnt	= cpu_to_be32(HS_EVNT_CLASS_ALL);

	req->hs_cb_id		= cpu_to_be32(0x1);

	req->hs_handle_ptr	= cpu_to_be32(0x1);
	req->hs_handle_data	= cpu_to_be32(0x0);

	return sizeof(*req);
}

static int hs_rpc_register_subs_res(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	printk("%s\n", __func__);
	uint32_t result;

	result = be32_to_cpu(*((uint32_t *)buffer));
	pr_debug("%s: request completed: 0x%x\n", __func__, result);

	return 0;
}

static int hs_cb_func(struct msm_rpc_client *client, void *buffer, int in_size)
{
	printk("%s\n", __func__);
	int rc = -1;

	struct rpc_request_hdr *hdr = buffer;

	hdr->type = be32_to_cpu(hdr->type);
	hdr->xid = be32_to_cpu(hdr->xid);
	hdr->rpc_vers = be32_to_cpu(hdr->rpc_vers);
	hdr->prog = be32_to_cpu(hdr->prog);
	hdr->vers = be32_to_cpu(hdr->vers);
	hdr->procedure = be32_to_cpu(hdr->procedure);

	process_hs_rpc_request(hdr->procedure,
			    (void *) (hdr + 1));

	msm_rpc_start_accepted_reply(client, hdr->xid,
				     RPC_ACCEPTSTAT_SUCCESS);
	rc = msm_rpc_send_accepted_reply(client, 0);
	if (rc) {
		pr_err("%s: sending reply failed: %d\n", __func__, rc);
		return rc;
	}

	return 0;
}

static int __init hs_rpc_cb_init(void)
{
	printk("%s\n", __func__);
	int rc = 0;

	/* version 2 is used in 7x30 */
	rpc_client = msm_rpc_register_client("hs",
			HS_RPC_PROG, HS_RPC_VERS, 0, hs_cb_func);


	if (IS_ERR(rpc_client)) {
		pr_err("%s: couldn't open rpc client with version 1 err %ld\n",
			 __func__, PTR_ERR(rpc_client));
		return PTR_ERR(rpc_client);
	}
	rc = msm_rpc_client_req(rpc_client, HS_SUBSCRIBE_SRVC_PROC,
				hs_rpc_register_subs_arg, NULL,
				hs_rpc_register_subs_res, NULL, -1);
	if (rc) {
		pr_err("%s: couldn't send rpc client request\n", __func__);
		msm_rpc_unregister_client(rpc_client);
	}

	return rc;
}

static int __devinit hs_rpc_init(void)
{
	printk("%s\n", __func__);
	int rc;
/*
	rc = hs_rpc_cb_init();
	if (rc)
		pr_err("%s: failed to initialize rpc client\n", __func__);
*/
	rc = msm_rpc_create_server(&hs_rpc_server);
	if (rc < 0)
		pr_err("%s: failed to create rpc server\n", __func__);

	return 0;
}

static void __devexit hs_rpc_deinit(void)
{
	if (rpc_client)
		msm_rpc_unregister_client(rpc_client);
}

static ssize_t msm_headset_print_name(struct switch_dev *sdev, char *buf)
{
	printk("%s\n", __func__);
	switch (switch_get_state(&hs->sdev)) {
	case NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case MSM_HEADSET:
		return sprintf(buf, "Headset\n");
	}
	return -EINVAL;
}

static int __devinit hs_probe(struct platform_device *pdev)
{
	int rc;
	struct input_dev *ipdev;
	printk("%s\n", __func__);

	hs = kzalloc(sizeof(struct msm_handset), GFP_KERNEL);
	if (!hs)
		return -ENOMEM;

	hs->sdev.name	= "h2w";
	hs->sdev.print_name = msm_headset_print_name;

	rc = switch_dev_register(&hs->sdev);
	if (rc)
		goto err_switch_dev_register;

	ipdev = input_allocate_device();
	if (!ipdev) {
		rc = -ENOMEM;
		goto err_alloc_input_dev;
	}
	input_set_drvdata(ipdev, hs);

	hs->ipdev = ipdev;

	if (pdev->dev.platform_data)
		ipdev->name = pdev->dev.platform_data;
	else
		ipdev->name	= DRIVER_NAME;

	ipdev->id.vendor	= 0x0001;
	ipdev->id.product	= 1;
	ipdev->id.version	= 1;

	input_set_capability(ipdev, EV_KEY, KEY_POWER);
	input_set_capability(ipdev, EV_KEY, KEY_END);

/*	input_set_capability(ipdev, EV_KEY, KEY_MEDIA);
	input_set_capability(ipdev, EV_SW, SW_HEADPHONE_INSERT);
	input_set_capability(ipdev, EV_KEY, KEY_SEND);
	input_set_capability(ipdev, EV_KEY, KEY_BACK);
	input_set_capability(ipdev, EV_KEY, KEY_MENU);
	input_set_capability(ipdev, EV_KEY, KEY_HOME);
	input_set_capability(ipdev, EV_KEY, KEY_CAMERA);
    input_set_capability(ipdev, EV_KEY, KEY_CHG_ST_NONE);
    input_set_capability(ipdev, EV_KEY, KEY_CHG_ST_OVP);
    input_set_capability(ipdev, EV_KEY, KEY_CHG_ST_OVC);
    input_set_capability(ipdev, EV_KEY, KEY_CHG_ST_OVD);
    input_set_capability(ipdev, EV_KEY, KEY_CHG_ST_EXP);
*/

	rc = input_register_device(ipdev);
	if (rc) {
		dev_err(&ipdev->dev,
				"hs_probe: input_register_device rc=%d\n", rc);
		goto err_reg_input_dev;
	}

	platform_set_drvdata(pdev, hs);

	rc = hs_rpc_init();
	if (rc)
		goto err_hs_rpc_init;

	printk("%s done\n", __func__);
	return 0;

err_hs_rpc_init:
	input_unregister_device(ipdev);
	ipdev = NULL;
err_reg_input_dev:
	input_free_device(ipdev);
err_alloc_input_dev:
	switch_dev_unregister(&hs->sdev);
err_switch_dev_register:
	kfree(hs);
	return rc;
}

static int __devexit hs_remove(struct platform_device *pdev)
{
	struct msm_handset *hs = platform_get_drvdata(pdev);

	input_unregister_device(hs->ipdev);
	switch_dev_unregister(&hs->sdev);
	kfree(hs);
	hs_rpc_deinit();
	return 0;
}

static struct platform_driver hs_driver = {
	.probe		= hs_probe,
	.remove		= __devexit_p(hs_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init hs_init(void)
{
	printk("%s\n", __func__);
	return platform_driver_register(&hs_driver);
}
late_initcall(hs_init);

static void __exit hs_exit(void)
{
	platform_driver_unregister(&hs_driver);
}
module_exit(hs_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msm-handset");

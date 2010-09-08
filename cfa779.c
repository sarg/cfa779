/*
    cfa779.c - CrystalFontz CFA-779 i2c LCD driver

    Copyright (C) 2004-07 HexView <max@hexview.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>

#include <linux/io.h>
#include <linux/input.h>
#include <linux/input-polldev.h>

#include <linux/crc-ccitt.h>

#define CFA779_INIT 255         /* Default value for stored params */
#define CFA779_MAX_CONTRAST 200 /* max contrast value */
#define CFA779_MAX_BACKLIGHT 100        /* max backlight value */
#define CFA779_MAX_CURSOR_STYLE 3       /* max cursor style value */
#define CFA779_NUM_COLUMNS  16  /* LCD columns */
#define CFA779_NUM_ROWS     2   /* LCD rows */
#define CFA779_NUM_KEYS     5   /* keypad keys */

#define POLL_INTERVAL_DEFAULT   100

/* insmod options */
static unsigned int debug = 0;
static unsigned int rawcmd = 0;


module_param (debug, int, 0);
MODULE_PARM_DESC (debug, "enable debug messages");
module_param (rawcmd, int, 0);
MODULE_PARM_DESC (rawcmd, "enable rawcmd interface");

MODULE_AUTHOR ("Max <max@hexview.com>");
MODULE_DESCRIPTION ("CrystalFontz CFA779 LCD Driver");
MODULE_LICENSE ("GPL");

static const unsigned short normal_i2c[] = { 0x20, I2C_CLIENT_END };

I2C_CLIENT_INSMOD_1 (cfa779);

/* ---------------------------------------------------------------------*/

s32
i2c_smbus_read_block_data (struct i2c_client *client, u8 command, u8 * values)
{
    union i2c_smbus_data data;
    int i;
    if (i2c_smbus_xfer (client->adapter, client->addr, client->flags,
                        I2C_SMBUS_READ, command, I2C_SMBUS_BLOCK_DATA, &data))
        return -1;
    else
      {
          for (i = 1; i <= data.block[0]; i++)
              values[i - 1] = data.block[i];
          return data.block[0];
      }
}

/* Each client has this additional data */
struct cfa779_data
{
    struct i2c_client *client;
    struct input_polled_dev *ipdev;
    unsigned short keymap[CFA779_NUM_KEYS];
    u16 nkeys;
    u8 backlight;               /* Stores last written value */
    u8 contrast;                /* Stores last written value */
    u8 cursor;                  /* Stores last written value */
};

static int cfa779_probe (struct i2c_client *client,
                         const struct i2c_device_id *id);
static int cfa779_remove (struct i2c_client *client);
static int cfa779_detect (struct i2c_client *client, int kind,
                          struct i2c_board_info *board_info);

struct i2c_device_id cfa779_idtable[] = {
    {"cfa779", 0},
    {}
};

MODULE_DEVICE_TABLE (i2c, cfa779_idtable);

/* This is the driver that will be inserted */
static struct i2c_driver cfa779_driver = {
    .class = I2C_CLASS_HWMON,
    .driver = {
               .owner = THIS_MODULE,
               .name = "cfa779",
               },
    .id_table = cfa779_idtable,
    .probe = cfa779_probe,
    .remove = cfa779_remove,

    .address_data = &addr_data,
    .detect = cfa779_detect
};


static void lcd_send_packet (struct i2c_client *client, u8 code, int len,
                             char *data);
static int lcd_check_reply (struct i2c_client *client, u8 code, int len,
                            char *buf);

static ssize_t cfa779_show_version (struct device *dev,
                                    struct device_attribute *attr, char *buf);
static ssize_t cfa779_show_keypad (struct device *dev,
                                   struct device_attribute *attr, char *buf);
static ssize_t cfa779_show_contrast (struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf);
static ssize_t cfa779_show_backlight (struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf);
static ssize_t cfa779_show_cursor_style (struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf);

static ssize_t cfa779_set_contrast (struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count);
static ssize_t cfa779_set_backlight (struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count);
static ssize_t cfa779_set_cursor_style (struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count);
static ssize_t cfa779_set_line1 (struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t count);
static ssize_t cfa779_set_line2 (struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t count);
static ssize_t cfa779_set_character (struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count);
static ssize_t cfa779_set_cursor_pos (struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count);
static ssize_t cfa779_set_rawcmd (struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count);

static u16
calc_crc (u8 * data, int len)
{
    return ~crc_ccitt (0xFFFF, data, len);
}


static DEVICE_ATTR (version, S_IRUGO, cfa779_show_version, NULL);
static DEVICE_ATTR (contrast, S_IWUSR | S_IRUGO, cfa779_show_contrast,
                    cfa779_set_contrast);
static DEVICE_ATTR (backlight, S_IWUSR | S_IRUGO, cfa779_show_backlight,
                    cfa779_set_backlight);
static DEVICE_ATTR (cursor_style, S_IWUSR | S_IRUGO, cfa779_show_cursor_style,
                    cfa779_set_cursor_style);
static DEVICE_ATTR (line1, S_IWUSR, NULL, cfa779_set_line1);
static DEVICE_ATTR (line2, S_IWUSR, NULL, cfa779_set_line2);
static DEVICE_ATTR (user_character, S_IWUSR, NULL, cfa779_set_character);
static DEVICE_ATTR (keypad, S_IRUGO, cfa779_show_keypad, NULL);
static DEVICE_ATTR (cursor_position, S_IWUSR, NULL, cfa779_set_cursor_pos);
static DEVICE_ATTR (rawcmd, S_IWUSR, NULL, cfa779_set_rawcmd);

/* receives reply, checks crc, optionally copies reply to buffer
returns reply length or 0 if error */
static int
lcd_check_reply (struct i2c_client *client, u8 code, int len, char *buf)
{
    u8 tb[256];
    int i;
    u16 crc;

    i = i2c_smbus_read_block_data (client, code, &tb[1]);
    tb[0] = i;
    if (buf != NULL)
        memcpy (buf, &tb[1], i);
    if (i < 3)
      {
          printk
              ("cfa779: No reply from LCD (cmd: 0x%02X, reply length: %d)\n",
               code, i);
          return 0;
      }
    crc = calc_crc (tb, i - 1);
    if ((tb[i - 1] != (crc & 0xFF)) || (tb[i] != ((crc >> 8) & 0xFF)))
      {
          printk ("cfa779: Received packet with invalid CRC (cmd: 0x%02X)\n",
                  code);
          return 0;
      }
    if ((len != -1) && ((len + 3) != i))
        printk ("cfa779: Invalid packet length: %d, expected: %d\n", len + 3,
                i);
    if ((tb[1] & 0xBF) != code)
        printk ("cfa779: cmd 0x%02X failed with code (0x%02X)\n", code,
                tb[1]);
    return i;
}

// code 8
static void
lcd_get_version (struct i2c_client *client, char *buf, int len)
{
    u8 tb[256];

    lcd_send_packet (client, 8, 0, NULL);
    if (lcd_check_reply (client, 8, -1, tb) != 0)
      {
          if (tb[0] >= 3)
              tb[0] -= 3;
          if (len >= tb[0])
              len = tb[0];
          memcpy (buf, &tb[1], len);
          buf[len] = 0;
      }
    else
        buf[0] = 0;

}

static ssize_t
cfa779_show_version (struct device *dev, struct device_attribute *attr,
                     char *buf)
{
    struct i2c_client *client = to_i2c_client (dev);
    char tb[128];
    memset (tb, 0, sizeof (tb));
    lcd_get_version (client, tb, sizeof (tb) - 1);
    return sprintf (buf, "cfa779 LCD Driver Version 1.1 (Hardware %s)\n", tb);
}

static ssize_t
cfa779_show_backlight (struct device *dev, struct device_attribute *attr,
                       char *buf)
{
    struct cfa779_data *data = i2c_get_clientdata (to_i2c_client (dev));
    return sprintf (buf, "%u\n", data->backlight);
}

static ssize_t
cfa779_show_contrast (struct device *dev, struct device_attribute *attr,
                      char *buf)
{
    struct cfa779_data *data = i2c_get_clientdata (to_i2c_client (dev));
    return sprintf (buf, "%u\n", data->contrast);
}

static ssize_t
cfa779_show_cursor_style (struct device *dev, struct device_attribute *attr,
                          char *buf)
{
    struct cfa779_data *data = i2c_get_clientdata (to_i2c_client (dev));
    return sprintf (buf, "%u\n", data->cursor);
}

// code 9
static ssize_t
cfa779_show_keypad (struct device *dev, struct device_attribute *attr,
                    char *buf)
{
    struct i2c_client *client = to_i2c_client (dev);
    u8 tb[256];
    int i, j, k;

    lcd_send_packet (client, 9, 0, NULL);
    i = lcd_check_reply (client, 9, -1, tb);

    if (i != 14)
        return 0;

    k = 64;
    for (j = 1; j < i - 2; j++)
        k += sprintf (&tb[k], "%u ", tb[j]);

    return sprintf (buf, "%s\n", &tb[64]);
}

// code 6
static ssize_t
cfa779_set_contrast (struct device *dev, struct device_attribute *attr,
                     const char *buf, size_t count)
{
    u8 vbyte;
    struct i2c_client *client = to_i2c_client (dev);
    struct cfa779_data *data = i2c_get_clientdata (client);
    unsigned long val = simple_strtoul (buf, NULL, 10);
    if (val > CFA779_MAX_CONTRAST)
        return -EINVAL;
    vbyte = val;
    lcd_send_packet (client, 6, 1, &vbyte);
//if (lcd_check_reply(client,6,0,NULL)!=0) 
    data->contrast = val;
    return count;
}

// code 5
static ssize_t
cfa779_set_cursor_style (struct device *dev, struct device_attribute *attr,
                         const char *buf, size_t count)
{
    u8 vbyte;
    struct i2c_client *client = to_i2c_client (dev);
    struct cfa779_data *data = i2c_get_clientdata (client);
    unsigned long val = simple_strtoul (buf, NULL, 10);
    if (val > CFA779_MAX_CURSOR_STYLE)
        return -EINVAL;
    vbyte = val;
    lcd_send_packet (client, 5, 1, &vbyte);
//if (lcd_check_reply(client,5,0,NULL)!=0)
    data->cursor = val;
    return count;
}

// code 4
static ssize_t
cfa779_set_cursor_pos (struct device *dev, struct device_attribute *attr,
                       const char *buf, size_t count)
{
    unsigned int x, y;
    u8 val[2];
    struct i2c_client *client = to_i2c_client (dev);

    if ((sscanf (buf, "%u %u", &y, &x) != 2) || (x > CFA779_NUM_COLUMNS)
        || (y >= CFA779_NUM_ROWS))
        return -EINVAL;

    val[0] = x;
    val[1] = y;

    lcd_send_packet (client, 4, 2, val);
//lcd_check_reply(client,4,0,NULL);
    return count;
}

// code 7
static ssize_t
cfa779_set_backlight (struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
    u8 vbyte;
    struct i2c_client *client = to_i2c_client (dev);
    struct cfa779_data *data = i2c_get_clientdata (client);
    unsigned long val = simple_strtoul (buf, NULL, 10);
    if (val > CFA779_MAX_BACKLIGHT)
        return -EINVAL;
    vbyte = val;
    lcd_send_packet (client, 7, 1, &vbyte);
//if (lcd_check_reply(client,7,0,NULL)!=0) 
    data->backlight = val;
    return count;
}

static ssize_t
cfa779_set_rawcmd (struct device *dev, struct device_attribute *attr,
                   const char *buf, size_t count)
{
    char val[16];
    char tb[256];
    int i, j, k;
    struct i2c_client *client = to_i2c_client (dev);
    size_t mycnt;

    mycnt = count;
    if (mycnt > 17)
        mycnt = 17;
    mycnt--;

    if (mycnt == 0) {
        lcd_send_packet (client, buf[0], mycnt, NULL);
    } else {
        memcpy (val, &buf[1], mycnt);
        lcd_send_packet (client, buf[0], mycnt, val);
    }
    i = lcd_check_reply (client, buf[0], -1, tb);

    k = 64;
    for (j = 0; j < i; j++)
        k += sprintf (&tb[k], "%02X ", tb[j] & 0xFF);
    printk ("Result(%d): %s\n", i, &tb[64]);
    return count;
}

// code 1
// code 2
static ssize_t
lcd_set_text (struct device *dev, const char *buf, size_t count, u8 line)
{
    char val[CFA779_NUM_COLUMNS];
    struct i2c_client *client = to_i2c_client (dev);
    size_t mycnt;

    memset (val, 0x20, sizeof (val));
    mycnt = count;
    if (mycnt > CFA779_NUM_COLUMNS)
        mycnt = CFA779_NUM_COLUMNS;
    memcpy (val, buf, mycnt);
    lcd_send_packet (client, line, 16, val);
//lcd_check_reply(client,line,0,NULL);
    return count;
}

static ssize_t
cfa779_set_line1 (struct device *dev, struct device_attribute *attr,
                  const char *buf, size_t count)
{
    return lcd_set_text (dev, buf, count, 1);
}

static ssize_t
cfa779_set_line2 (struct device *dev, struct device_attribute *attr,
                  const char *buf, size_t count)
{
    return lcd_set_text (dev, buf, count, 2);
}

// code 3
/* defines user character, fyrst byte is character code (0..7),
other 8 bytes are bitmasks */
static ssize_t
cfa779_set_character (struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
    u8 val[9];
    unsigned int bmp[9];
    int i;
    struct i2c_client *client = to_i2c_client (dev);

    if (sscanf (buf, "%u %u %u %u %u %u %u %u %u", &bmp[0], &bmp[1],
                &bmp[2], &bmp[3], &bmp[4], &bmp[5], &bmp[6], &bmp[7],
                &bmp[8]) != 9)
        return -EINVAL;

    for (i = 0; i < 9; i++)
        val[i] = bmp[i] & 0xFF;

    lcd_send_packet (client, 3, 9, val);
//lcd_check_reply(client,3,0,NULL);
    return count;
}


void
lcd_send_packet (struct i2c_client *client, u8 idx, int len, char *data)
{
    u8 val[24];
    u16 crc;
    if (len > 0x10)
        len = 0x10;

    val[0] = idx;
    memcpy (&val[2], data, len++);
    val[1] = ++len;

    crc = calc_crc (val, len);

    val[len] = crc & 0xFF;
    val[len + 1] = (crc >> 8) & 0xFF;
    i2c_smbus_write_block_data (client, idx, len, &val[2]);
}


static int
cfa779_detect (struct i2c_client *new_client, int kind,
               struct i2c_board_info *info)
{
    struct i2c_adapter *adapter = new_client->adapter;

    if (!i2c_check_functionality
        (adapter, I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WRITE_WORD_DATA))
        return -ENODEV;

    /*
     * Now we do the remaining detection. A negative kind means that
     * the driver was loaded with no force parameter (default), so we
     * must both detect and identify the chip. A zero kind means that
     * the driver was loaded with the force parameter, the detection
     * step shall be skipped. A positive kind means that the driver
     * was loaded with the force parameter and a given kind of chip is
     * requested, so both the detection and the identification steps
     * are skipped.
     */

    if (kind == 0)
        kind = cfa779;

    if (kind < 0)
      {                         /* detection and identification */
          lcd_send_packet (new_client, 0, 0, NULL);
          if (lcd_check_reply (new_client, 0, 0, NULL) == 0)
              return -ENODEV;
          kind = cfa779;
      }

    strncpy (info->type, "cfa779", I2C_NAME_SIZE);

    return 0;
}

static struct cfa779_data cdata = {
    .keymap = {
        [0] = KEY_UP,
        [1] = KEY_DOWN,
        [2] = KEY_LEFT,
        [3] = KEY_RIGHT,
        [4] = KEY_ENTER,
    },
};

u8 kbd_press[] = {3, 4, 1, 2, 5};
u8 kbd_release[] = {8, 9, 6, 7, 10};

static void cfa779_poll(struct input_polled_dev *ipdev)
{
    struct cfa779_data *data = ipdev->private;
    struct input_dev *idev = ipdev->input;

    u8 tb[256];
    int i;

    lcd_send_packet(data->client, 9, 0, NULL);
    i = lcd_check_reply(data->client, 9, -1, tb);

    if (i != 14) return;

    for (i = 0; i < idev->keycodemax; i++) {
        if (tb[kbd_press[i]+1]) {
            printk("Pressed %d\n", data->keymap[i]);
            input_report_key(idev, data->keymap[i], 1);
        }

        if (tb[kbd_release[i]+1]) {
            printk("Released %d\n", data->keymap[i]);
            input_report_key(idev, data->keymap[i], 0);
        }
    }
    input_sync(idev);
}

static int cfa779_register_sysfs(struct i2c_client *client) 
{
    struct device *dev = &client->dev;
    int err;

    /* Register sysfs hooks */
    if ((err = device_create_file (dev, &dev_attr_version)))
        goto fail1;
    if ((err = device_create_file (dev, &dev_attr_contrast)))
        goto fail2;
    if ((err = device_create_file (dev, &dev_attr_backlight)))
        goto fail3;
    if ((err = device_create_file (dev, &dev_attr_line1)))
        goto fail4;
    if ((err = device_create_file (dev, &dev_attr_line2)))
        goto fail5;
    if ((err = device_create_file (dev, &dev_attr_keypad)))
        goto fail6;
    if ((err = device_create_file (dev, &dev_attr_user_character)))
        goto fail7;
    if ((err = device_create_file (dev, &dev_attr_cursor_style)))
        goto fail8;
    if ((err = device_create_file (dev, &dev_attr_cursor_position)))
        goto fail9;

    if (rawcmd != 0)
        if ((err = device_create_file (dev, &dev_attr_rawcmd)))
            goto fail10;

    return 0;
fail10:
    device_remove_file (dev, &dev_attr_cursor_position);
fail9:
    device_remove_file (dev, &dev_attr_cursor_style);
fail8:
    device_remove_file (dev, &dev_attr_user_character);
fail7:
    device_remove_file (dev, &dev_attr_keypad);
fail6:
    device_remove_file (dev, &dev_attr_line2);
fail5:
    device_remove_file (dev, &dev_attr_line1);
fail4:
    device_remove_file (dev, &dev_attr_backlight);
fail3:
    device_remove_file (dev, &dev_attr_contrast);
fail2:
    device_remove_file (dev, &dev_attr_version);
fail1:
    return err;
}

static void cfa779_unregister_sysfs(struct i2c_client *client)
{
    struct device *dev = &client->dev;

    if (rawcmd != 0) 
        device_remove_file (dev, &dev_attr_rawcmd);
    device_remove_file (dev, &dev_attr_cursor_position);
    device_remove_file (dev, &dev_attr_cursor_style);
    device_remove_file (dev, &dev_attr_user_character);
    device_remove_file (dev, &dev_attr_keypad);
    device_remove_file (dev, &dev_attr_line2);
    device_remove_file (dev, &dev_attr_line1);
    device_remove_file (dev, &dev_attr_backlight);
    device_remove_file (dev, &dev_attr_contrast);
    device_remove_file (dev, &dev_attr_version);
}


/* This function is called by i2c_probe */
static int
cfa779_probe (struct i2c_client *client, const struct i2c_device_id *id)
{
    struct cfa779_data *data = &cdata;
    s32 b;
    int err = 0;
    int i;
    struct input_polled_dev *ipdev;
    struct input_dev *idev;
    char buf[CFA779_NUM_COLUMNS + 1];

    struct device *dev = &client->dev;

    if (!i2c_check_functionality (client->adapter, I2C_FUNC_SMBUS_BYTE_DATA |
                                  I2C_FUNC_SMBUS_WRITE_WORD_DATA))
        return -ENODEV;

    data->client = client;
    i2c_set_clientdata (client, data);

    b = i2c_smbus_read_byte_data (client, 0x20);
    pr_debug("cfa779: LCD Type = 0x%02X\n", b);

    strncpy (client->name, "cfa779", I2C_NAME_SIZE);

    data->backlight = CFA779_INIT;
    data->contrast = CFA779_INIT;
    data->cursor = CFA779_INIT;

    ipdev = input_allocate_polled_device();
    if (!ipdev)
        return -ENOMEM;

    data->ipdev = ipdev;

    ipdev->poll = cfa779_poll;
    ipdev->poll_interval = POLL_INTERVAL_DEFAULT;
    ipdev->private = data;

    idev = ipdev->input;
    idev->name = "cfa779 buttons";
    idev->phys = "cfa779/input0";
    idev->id.bustype = BUS_HOST;
    idev->dev.parent = &client->dev;

    set_bit(EV_KEY, idev->evbit);

    idev->keycode = data->keymap;
    idev->keycodesize = sizeof(data->keymap[0]);
    idev->keycodemax = 5;

    for (i = 0; i < idev->keycodemax; i++)
        if (data->keymap[i]) {
            set_bit(data->keymap[i], idev->keybit);
        }

    err = input_register_polled_device(ipdev);
    if (err) goto exit_free;

    err = cfa779_register_sysfs(client);
    if (err) {
        dev_err(&client->dev, "cfa779 registering sysfs failed \n");
        goto exit_unregister;
    }


    /*??? Reset the cfa779 chip */
    i2c_smbus_write_byte_data (client, 0, 1);

    lcd_set_text (dev, "cfa779 driver OK", 16, 1);

    sprintf (buf, "LCD ");
    lcd_get_version (client, &buf[4], CFA779_NUM_COLUMNS - 4);

    lcd_set_text (dev, buf, strlen (buf), 2);

    return 0;

  exit_unregister:
    input_unregister_polled_device(ipdev);
  exit_free:
    input_free_polled_device(ipdev);
    return err;
}

static int
cfa779_remove (struct i2c_client *client)
{
    struct cfa779_data *data = i2c_get_clientdata(client);

    cfa779_unregister_sysfs(client);

    input_unregister_polled_device(data->ipdev);
    input_free_polled_device(data->ipdev);

    lcd_set_text (&client->dev, "Shutdown", 8, 1);
    lcd_set_text (&client->dev, "Finished", 8, 2);

    return 0;
}

/* ------------------------------------------------------------ */


static int __init
cfa779_init (void)
{
    return i2c_add_driver (&cfa779_driver);
}

static void __exit
cfa779_exit (void)
{
    i2c_del_driver (&cfa779_driver);
}

module_init (cfa779_init);
module_exit (cfa779_exit);

import { describe, it, expect, beforeEach, vi } from 'vitest'

// Mock pinia BEFORE importing badge functions using vi.hoisted
const { mockConfig } = vi.hoisted(() => ({
  mockConfig: {
    mdns: '',
    wifi_ssid: '',
    wifi_ssid2: '',
    http_post: '',
    http_get: '',
    influxdb2_push: '',
    mqtt_push: ''
  }
}))

vi.mock('@/modules/pinia', () => ({
  config: mockConfig
}))

import {
  deviceBadge,
  deviceSettingBadge,
  deviceMdnsBadge,
  deviceWifiBadge,
  deviceWifi1Badge,
  deviceWifi2Badge,
  pushBadge,
  pushHttpPostBadge,
  pushHttpGetBadge,
  pushHttpInfluxdb2Badge,
  pushHttpMqttBadge,
  deviceHardwareBadge,
  devicePidBadge,
  pushBluetoothBadge,
  pushInfluxdb2Badge
} from '../badge.js'
import { config } from '@/modules/pinia'

describe('badge.js', () => {
  beforeEach(() => {
    // Reset all config properties to empty before each test
    config.mdns = ''
    config.wifi_ssid = ''
    config.wifi_ssid2 = ''
    config.http_post = ''
    config.http_get = ''
    config.influxdb2_push = ''
    config.mqtt_push = ''
  })

  describe('deviceMdnsBadge()', () => {
    it('returns 1 when mdns is empty', () => {
      config.mdns = ''
      expect(deviceMdnsBadge()).toBe(1)
    })

    it('returns 0 when mdns is configured', () => {
      config.mdns = 'mydevice'
      expect(deviceMdnsBadge()).toBe(0)
    })
  })

  describe('deviceWifi1Badge()', () => {
    it('returns 1 when wifi_ssid is empty', () => {
      config.wifi_ssid = ''
      expect(deviceWifi1Badge()).toBe(1)
    })

    it('returns 0 when wifi_ssid is configured', () => {
      config.wifi_ssid = 'MyWiFi'
      expect(deviceWifi1Badge()).toBe(0)
    })
  })

  describe('deviceWifi2Badge()', () => {
    it('returns 1 when both wifi_ssid and wifi_ssid2 are empty', () => {
      config.wifi_ssid = ''
      config.wifi_ssid2 = ''
      expect(deviceWifi2Badge()).toBe(1)
    })

    it('returns 0 when wifi_ssid is configured', () => {
      config.wifi_ssid = 'MyWiFi'
      config.wifi_ssid2 = ''
      expect(deviceWifi2Badge()).toBe(0)
    })
  })

  describe('deviceWifiBadge()', () => {
    it('returns 1 when both wifi networks are empty', () => {
      config.wifi_ssid = ''
      config.wifi_ssid2 = ''
      expect(deviceWifiBadge()).toBe(1)
    })

    it('returns 0 when at least one wifi network is configured', () => {
      config.wifi_ssid = 'MyNetwork'
      config.wifi_ssid2 = ''
      expect(deviceWifiBadge()).toBe(0)
    })
  })

  describe('deviceSettingBadge()', () => {
    it('returns badge count from mdns check', () => {
      config.mdns = ''
      expect(deviceSettingBadge()).toBe(1)
    })

    it('returns 0 when mdns is configured', () => {
      config.mdns = 'device'
      expect(deviceSettingBadge()).toBe(0)
    })
  })

  describe('deviceBadge()', () => {
    it('returns total of setting and wifi badges', () => {
      const result = deviceBadge()
      expect(typeof result).toBe('number')
    })

    it('returns 0 when all settings are configured', () => {
      config.mdns = 'device'
      config.wifi_ssid = 'MyWiFi'
      config.wifi_ssid2 = ''
      expect(deviceBadge()).toBe(0)
    })

    it('returns 2 when both mdns and wifi are empty', () => {
      config.mdns = ''
      config.wifi_ssid = ''
      config.wifi_ssid2 = ''
      expect(deviceBadge()).toBe(2)
    })
  })

  describe('Push Badge Functions', () => {
    it('pushHttpPostBadge returns 1 when no targets configured', () => {
      expect(pushHttpPostBadge()).toBe(1)
    })

    it('pushHttpPostBadge returns 0 when targets are configured', () => {
      config.http_post = 'http://api.example.com'
      expect(pushHttpPostBadge()).toBe(0)
    })

    it('pushHttpGetBadge returns badge based on target count', () => {
      expect(pushHttpGetBadge()).toBe(1)
    })

    it('pushHttpInfluxdb2Badge returns badge based on target count', () => {
      expect(pushHttpInfluxdb2Badge()).toBe(1)
    })

    it('pushHttpMqttBadge returns badge based on target count', () => {
      expect(pushHttpMqttBadge()).toBe(1)
    })

    it('pushBluetoothBadge returns badge based on target count', () => {
      expect(pushBluetoothBadge()).toBe(1)
    })

    it('pushInfluxdb2Badge returns badge based on target count', () => {
      expect(pushInfluxdb2Badge()).toBe(1)
    })
  })

  describe('pushBadge()', () => {
    it('returns total of all push target badges', () => {
      const result = pushBadge()
      expect(typeof result).toBe('number')
    })

    it('returns 0 when all push targets are configured', () => {
      config.http_post = 'http://post.example.com'
      config.http_get = 'http://get.example.com'
      config.influxdb2_push = 'http://influx.example.com'
      config.mqtt_push = 'mqtt://broker.example.com'
      expect(pushBadge()).toBe(0)
    })

    it('returns 4 when no push targets are configured', () => {
      expect(pushBadge()).toBe(4)
    })
  })

  describe('Hard-coded Badge Functions', () => {
    it('deviceHardwareBadge always returns 0', () => {
      expect(deviceHardwareBadge()).toBe(0)
    })

    it('devicePidBadge always returns 0', () => {
      expect(devicePidBadge()).toBe(0)
    })
  })
})

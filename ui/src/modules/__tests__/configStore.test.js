import { describe, it, expect, beforeEach, vi } from 'vitest'
import { createPinia, setActivePinia } from 'pinia'
import { useConfigStore, saveConfigState } from '../configStore'
// Note: useGlobalStore is mocked below - tests use mockGlobal directly

// Shared mock for the global store — always the same object so configStore's
// lazy singleton (getGlobalStore) and test assertions see the same reference.
const mockGlobal = {
  disabled: false,
  messageError: '',
  messageSuccess: '',
  messageInfo: '',
  configChanged: false,
  clearMessages: vi.fn()
}

vi.mock('@/modules/globalStore', () => ({
  useGlobalStore: vi.fn(() => mockGlobal)
}))

// Mock the espframework components
vi.mock('@mp-se/espframework-ui-components', () => ({
  logDebug: vi.fn(),
  logError: vi.fn(),
  logInfo: vi.fn(),
  sharedHttpClient: {
    getJson: vi.fn(),
    postJson: vi.fn(),
    request: vi.fn(),
    restart: vi.fn()
  }
}))

describe('configStore', () => {
  beforeEach(async () => {
    const pinia = createPinia()
    setActivePinia(pinia)
    vi.clearAllMocks()
    // Reset baseline by creating a fresh store instance and saving its state
    const freshConfig = useConfigStore()
    await saveConfigState(freshConfig)
    // Reset mock global state between tests
    mockGlobal.disabled = false
    mockGlobal.messageError = ''
    mockGlobal.messageSuccess = ''
    mockGlobal.messageInfo = ''
    mockGlobal.configChanged = false
    mockGlobal.clearMessages = vi.fn()
  })

  describe('state initialization', () => {
    it('initializes with empty state', () => {
      const config = useConfigStore()
      
      expect(config.id).toBe('')
      expect(config.mdns).toBe('')
      expect(config.temp_format).toBe('')
      expect(config.ota_url).toBe('')
      expect(config.restart_interval).toBe(0)
      expect(config.wifi_portal_timeout).toBe(0)
    })

    it('has all required config fields', () => {
      const config = useConfigStore()
      
      // Device fields
      expect(config).toHaveProperty('id')
      expect(config).toHaveProperty('mdns')
      expect(config).toHaveProperty('temp_format')
      
      // WiFi fields
      expect(config).toHaveProperty('wifi_ssid')
      expect(config).toHaveProperty('wifi_ssid2')
      expect(config).toHaveProperty('wifi_pass')
      expect(config).toHaveProperty('wifi_pass2')
      
      // HTTP POST fields
      expect(config).toHaveProperty('http_post_target')
      expect(config).toHaveProperty('http_post_header1')
      expect(config).toHaveProperty('http_post_header2')
      
      // InfluxDB fields
      expect(config).toHaveProperty('influxdb2_target')
      expect(config).toHaveProperty('influxdb2_bucket')
      expect(config).toHaveProperty('influxdb2_org')
      expect(config).toHaveProperty('influxdb2_token')
      
      // MQTT fields
      expect(config).toHaveProperty('mqtt_target')
      expect(config).toHaveProperty('mqtt_port')
      expect(config).toHaveProperty('mqtt_user')
      expect(config).toHaveProperty('mqtt_pass')
      
      // Hardware fields
      expect(config).toHaveProperty('fridge_sensor_id')
      expect(config).toHaveProperty('beer_sensor_id')
      expect(config).toHaveProperty('beer_ble_sensor_id')
      expect(config).toHaveProperty('fridge_sensor_offset')
      expect(config).toHaveProperty('beer_sensor_offset')
      expect(config).toHaveProperty('enable_cooling')
      expect(config).toHaveProperty('enable_heating')
      expect(config).toHaveProperty('invert_pins')
      
      // BLE fields
      expect(config).toHaveProperty('ble_push_enabled')
      expect(config).toHaveProperty('ble_scan_enabled')
      expect(config).toHaveProperty('ble_sensor_valid_time')
    })
  })

  describe('state mutations', () => {
    it('updates string fields', () => {
      const config = useConfigStore()
      
      config.id = 'test-device'
      config.mdns = 'mybrewer'
      config.temp_format = 'C'
      
      expect(config.id).toBe('test-device')
      expect(config.mdns).toBe('mybrewer')
      expect(config.temp_format).toBe('C')
    })

    it('updates numeric fields', () => {
      const config = useConfigStore()
      
      config.restart_interval = 3600
      config.mqtt_port = 1883
      config.beer_sensor_offset = 0.5
      config.fridge_sensor_offset = -0.25
      
      expect(config.restart_interval).toBe(3600)
      expect(config.mqtt_port).toBe(1883)
      expect(config.beer_sensor_offset).toBe(0.5)
      expect(config.fridge_sensor_offset).toBe(-0.25)
    })

    it('updates boolean fields', () => {
      const config = useConfigStore()
      
      config.dark_mode = true
      config.enable_cooling = true
      config.enable_heating = false
      config.invert_pins = true
      config.ble_push_enabled = true
      config.ble_scan_enabled = false
      
      expect(config.dark_mode).toBe(true)
      expect(config.enable_cooling).toBe(true)
      expect(config.enable_heating).toBe(false)
      expect(config.invert_pins).toBe(true)
      expect(config.ble_push_enabled).toBe(true)
      expect(config.ble_scan_enabled).toBe(false)
    })
  })

  describe('load action', () => {
    it('loads configuration from API', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      
      const mockData = {
        id: 'device-123',
        mdns: 'mybrewer',
        temp_format: 'C',
        ota_url: 'http://example.com/ota',
        restart_interval: 3600,
        wifi_portal_timeout: 300,
        wifi_connect_timeout: 30,
        wifi_ssid: 'MyWiFi',
        wifi_ssid2: '',
        wifi_pass: 'password',
        wifi_pass2: '',
        push_timeout: 60,
        http_post_target: 'http://example.com/post',
        http_post_header1: '',
        http_post_header2: '',
        http_post2_target: '',
        http_post2_header1: '',
        http_post2_header2: '',
        http_get_target: '',
        http_get_header1: '',
        http_get_header2: '',
        influxdb2_target: 'http://influx:8086',
        influxdb2_bucket: 'mybucket',
        influxdb2_org: 'myorg',
        influxdb2_token: 'mytoken',
        mqtt_target: 'mqtt.example.com',
        mqtt_port: 1883,
        mqtt_user: 'user',
        mqtt_pass: 'pass',
        dark_mode: true,
        fridge_sensor_id: 'sensor1',
        beer_sensor_id: 'sensor2',
        beer_ble_sensor_id: 'ble1',
        fridge_sensor_offset: 0.5,
        beer_sensor_offset: -0.25,
        controller_mode: 'f',
        target_temperature: 20,
        enable_cooling: true,
        enable_heating: false,
        invert_pins: false,
        ble_push_enabled: true,
        ble_scan_enabled: true,
        ble_sensor_valid_time: 300
      }
      
      http.getJson.mockResolvedValue(mockData)
      
      const config = useConfigStore()
      const result = await config.load()
      
      expect(result).toBe(true)
      expect(http.getJson).toHaveBeenCalledWith('api/config')
      expect(config.id).toBe('device-123')
      expect(config.mdns).toBe('mybrewer')
      expect(config.dark_mode).toBe(true)
      expect(config.fridge_sensor_offset).toBe(0.5)
    })

    it('handles load errors gracefully', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      
      http.getJson.mockRejectedValue(new Error('Network error'))
      
      const config = useConfigStore()
      const result = await config.load()
      
      expect(result).toBe(false)
    })
  })

  describe('sendConfig action', () => {
    it('sends config changes to API', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      
      http.postJson.mockResolvedValue({ success: true })
      
      const config = useConfigStore()
      config.mdns = 'newname'
      config.dark_mode = true
      
      const result = await config.sendConfig()
      
      // Should succeed even with no changes tracked (depends on saveConfigState being called)
      expect(result).toBe(true)
    })

    it('returns true when no changes pending', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      
      http.postJson.mockResolvedValue({ success: true })
      
      const config = useConfigStore()
      // Without calling saveConfigState, there are no tracked changes
      // So sendConfig returns true without calling postJson
      const result = await config.sendConfig()
      
      expect(result).toBe(true)
      // The HTTP client should not be called since there are no changes
      expect(http.postJson).not.toHaveBeenCalled()
    })

    it('detects and sends changes after loading config', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      
      // Mock load response
      const mockData = {
        id: 'device-123',
        mdns: 'mybrewer',
        temp_format: 'C',
        ota_url: 'http://example.com/ota',
        restart_interval: 3600,
        wifi_portal_timeout: 300,
        wifi_connect_timeout: 30,
        wifi_ssid: 'MyWiFi',
        wifi_ssid2: '',
        wifi_pass: 'password',
        wifi_pass2: '',
        push_timeout: 60,
        http_post_target: 'http://example.com/post',
        http_post_header1: '',
        http_post_header2: '',
        http_post2_target: '',
        http_post2_header1: '',
        http_post2_header2: '',
        http_get_target: '',
        http_get_header1: '',
        http_get_header2: '',
        influxdb2_target: 'http://influx:8086',
        influxdb2_bucket: 'mybucket',
        influxdb2_org: 'myorg',
        influxdb2_token: 'mytoken',
        mqtt_target: 'mqtt.example.com',
        mqtt_port: 1883,
        mqtt_user: 'user',
        mqtt_pass: 'pass',
        dark_mode: true,
        fridge_sensor_id: 'sensor1',
        beer_sensor_id: 'original-beer-sensor',
        beer_ble_sensor_id: 'ble1',
        fridge_sensor_offset: 0.5,
        beer_sensor_offset: -0.25,
        controller_mode: 'f',
        target_temperature: 20,
        enable_cooling: true,
        enable_heating: false,
        invert_pins: false,
        ble_push_enabled: true,
        ble_scan_enabled: true,
        ble_sensor_valid_time: 300
      }
      
      http.getJson.mockResolvedValue(mockData)
      http.postJson.mockResolvedValue({ success: true })
      
      const config = useConfigStore()
      
      // Load config from API
      const loadResult = await config.load()
      expect(loadResult).toBe(true)
      expect(config.beer_sensor_id).toBe('original-beer-sensor')
      
      // Change a field
      config.beer_sensor_id = '2817557997210309'
      
      // Send config - should detect the change and POST it
      const sendResult = await config.sendConfig()
      expect(sendResult).toBe(true)
      expect(http.postJson).toHaveBeenCalledWith('api/config', expect.objectContaining({
        beer_sensor_id: '2817557997210309'
      }))
    })
  })

  describe('restart action', () => {
    it('sends restart request to device', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')

      http.restart.mockResolvedValue({
        success: true,
        json: { status: true, message: 'Device restarting' }
      })

      const config = useConfigStore()

      config.mdns = 'mybrewer'
      await config.restart()

      expect(http.restart).toHaveBeenCalledWith('mybrewer', { redirectDelayMs: 8000 })
    })

    it('sets success message with redirect info', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      const global = mockGlobal
      http.restart.mockResolvedValue({
        success: true,
        json: { status: true, message: 'Restarting' }
      })
      const config = useConfigStore()
      config.mdns = 'mybrewer'

      await config.restart()

      expect(global.messageSuccess).toContain('mybrewer')
    })

    it('sets error when response has json but status is false', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      const global = mockGlobal
      http.restart.mockResolvedValue({
        success: true,
        json: { status: false, message: 'Device busy' }
      })

      const config = useConfigStore()
      await config.restart()

      expect(global.messageError).toContain('Device busy')
    })

    it('uses fallback error text when json message is empty', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      const global = mockGlobal
      http.restart.mockResolvedValue({
        success: true,
        json: { status: false, message: '' }
      })

      const config = useConfigStore()
      await config.restart()

      expect(global.messageError).toContain('Failed to restart device')
    })

    it('sets error when success is false (no json)', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      const global = mockGlobal
      http.restart.mockResolvedValue({
        success: false,
        json: null
      })

      const config = useConfigStore()
      await config.restart()

      expect(global.messageError).toContain('Failed to request restart')
    })

    it('handles restart errors gracefully', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')

      http.restart.mockRejectedValue(new Error('Connection error'))

      const config = useConfigStore()

      config.mdns = 'mybrewer'

      // Should not throw even if restart fails
      await expect(config.restart()).resolves.toBeUndefined()
    })

    it('sets error message on restart exception', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      const global = mockGlobal
      http.restart.mockRejectedValue(new Error('Connection error'))

      const config = useConfigStore()
      await config.restart()

      expect(global.messageError).toContain('Failed to do restart')
    })
  })

  describe('WiFi scanning actions', () => {
    it('sends WiFi scan request', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      
      http.request.mockResolvedValue({ success: true })
      
      const config = useConfigStore()
      const result = await config.sendWifiScan()
      
      expect(result).toBe(true)
      expect(http.request).toHaveBeenCalledWith('api/wifi')
    })

    it('handles WiFi scan errors', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      
      http.request.mockRejectedValue(new Error('Request failed'))
      
      const config = useConfigStore()
      const result = await config.sendWifiScan()
      
      expect(result).toBe(false)
    })

    it('gets WiFi scan status', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      
      http.getJson.mockResolvedValue({ status: false, success: true, networks: [] })
      
      const config = useConfigStore()
      const result = await config.getWifiScanStatus()
      
      expect(result.success).toBe(true)
      expect(result.data).toHaveProperty('networks')
    })
  })

  describe('Sensor scanning actions', () => {
    it('sends sensor scan request', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      
      http.request.mockResolvedValue({ success: true })
      
      const config = useConfigStore()
      const result = await config.sendSensorScan()
      
      expect(result).toBe(true)
      expect(http.request).toHaveBeenCalledWith('api/sensor')
    })

    it('gets sensor scan status', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      
      http.getJson.mockResolvedValue({ status: false, success: true })
      
      const config = useConfigStore()
      const result = await config.getSensorScanStatus()
      
      expect(result.success).toBe(true)
    })
  })

  describe('saveAll action', () => {
    it('calls sendConfig during saveAll', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')

      http.postJson.mockResolvedValue({ success: true })

      const config = useConfigStore()
      config.dark_mode = true

      // Spy on sendConfig
      const sendConfigSpy = vi.spyOn(config, 'sendConfig')

      await config.saveAll()

      expect(sendConfigSpy).toHaveBeenCalled()
    })

    it('completes without throwing', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')

      http.postJson.mockResolvedValue({ success: true })

      const config = useConfigStore()
      config.dark_mode = true

      // Should not throw
      await expect(config.saveAll()).resolves.toBeUndefined()
    })

    it('sets error message when sendConfig returns false', async () => {
      const config = useConfigStore()
      const global = mockGlobal
      vi.spyOn(config, 'sendConfig').mockResolvedValue(false)

      await config.saveAll()

      expect(global.messageError).toContain('Failed to store configuration')
    })

    it('sets error message when sendConfig throws', async () => {
      const config = useConfigStore()
      const global = mockGlobal
      vi.spyOn(config, 'sendConfig').mockRejectedValue(new Error('Unexpected error'))

      await config.saveAll()

      expect(global.messageError).toContain('Failed to save configuration')
    })

    it('sets success message when save completes', async () => {
      const config = useConfigStore()
      const global = mockGlobal
      vi.spyOn(config, 'sendConfig').mockResolvedValue(true)

      await config.saveAll()

      expect(global.messageSuccess).toContain('Configuration has been saved')
    })

    it('establishes new baseline after successful save', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      
      // Mock load response
      const mockData = {
        id: 'device-123',
        mdns: 'mybrewer',
        temp_format: 'C',
        ota_url: 'http://example.com/ota',
        restart_interval: 3600,
        wifi_portal_timeout: 300,
        wifi_connect_timeout: 30,
        wifi_ssid: 'MyWiFi',
        wifi_ssid2: '',
        wifi_pass: 'password',
        wifi_pass2: '',
        push_timeout: 60,
        http_post_target: 'http://example.com/post',
        http_post_header1: '',
        http_post_header2: '',
        http_post2_target: '',
        http_post2_header1: '',
        http_post2_header2: '',
        http_get_target: '',
        http_get_header1: '',
        http_get_header2: '',
        influxdb2_target: 'http://influx:8086',
        influxdb2_bucket: 'mybucket',
        influxdb2_org: 'myorg',
        influxdb2_token: 'mytoken',
        mqtt_target: 'mqtt.example.com',
        mqtt_port: 1883,
        mqtt_user: 'user',
        mqtt_pass: 'pass',
        dark_mode: false,
        fridge_sensor_id: 'sensor1',
        beer_sensor_id: 'original-beer-sensor',
        beer_ble_sensor_id: 'ble1',
        fridge_sensor_offset: 0.5,
        beer_sensor_offset: -0.25,
        controller_mode: 'f',
        target_temperature: 20,
        enable_cooling: true,
        enable_heating: false,
        invert_pins: false,
        ble_push_enabled: true,
        ble_scan_enabled: true,
        ble_sensor_valid_time: 300
      }
      
      http.getJson.mockResolvedValue(mockData)
      http.postJson.mockResolvedValue({ success: true })
      
      const config = useConfigStore()
      
      // Load config
      await config.load()
      expect(config.dark_mode).toBe(false)
      
      // Change and save
      config.dark_mode = true
      await config.saveAll()
      
      // Verify change was sent
      expect(http.postJson).toHaveBeenCalledWith('api/config', expect.objectContaining({
        dark_mode: true
      }))
      
      // Now the baseline should be reset, so no changes detected yet
      http.postJson.mockClear()
      const sendResult = await config.sendConfig()
      expect(sendResult).toBe(true)
      expect(http.postJson).not.toHaveBeenCalled()
      
      // But if we change something else, it should be detected
      config.target_temperature = 25
      const sendResult2 = await config.sendConfig()
      expect(sendResult2).toBe(true)
      expect(http.postJson).toHaveBeenCalledWith('api/config', expect.objectContaining({
        target_temperature: 25
      }))
    })
  })

  describe('global store integration', () => {
    it('gets global store instance', async () => {
      const config = useConfigStore()
      const global = mockGlobal
      
      // Verify that global store exists and has expected properties
      expect(global).toBeDefined()
      expect(global).toHaveProperty('disabled')
      expect(global).toHaveProperty('messageError')
      expect(global).toHaveProperty('messageSuccess')
    })

    it('calls restart with correct mdns', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      
      http.restart.mockResolvedValue({
        success: true,
        json: { status: true, message: 'OK' }
      })
      
      const config = useConfigStore()
      
      config.mdns = 'test-device'
      await config.restart()
      
      expect(http.restart).toHaveBeenCalledWith('test-device', { redirectDelayMs: 8000 })
    })

    it('handles restart API call errors', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      
      http.restart.mockRejectedValue(new Error('Connection error'))
      
      const config = useConfigStore()
      
      config.mdns = 'test'
      
      // Should not throw even if restart fails
      await expect(config.restart()).resolves.toBeUndefined()
    })
  })

  describe('WiFi configuration fields', () => {
    it('stores WiFi credentials', () => {
      const config = useConfigStore()
      
      config.wifi_ssid = 'MyNetwork'
      config.wifi_pass = 'password123'
      config.wifi_ssid2 = 'BackupNetwork'
      config.wifi_pass2 = 'backup456'
      config.wifi_portal_timeout = 120
      config.wifi_connect_timeout = 30
      
      expect(config.wifi_ssid).toBe('MyNetwork')
      expect(config.wifi_pass).toBe('password123')
      expect(config.wifi_ssid2).toBe('BackupNetwork')
      expect(config.wifi_pass2).toBe('backup456')
      expect(config.wifi_portal_timeout).toBe(120)
      expect(config.wifi_connect_timeout).toBe(30)
    })
  })

  describe('HTTP configuration fields', () => {
    it('stores multiple HTTP POST targets with headers', () => {
      const config = useConfigStore()
      
      config.http_post_target = 'http://server1.com/data'
      config.http_post_header1 = 'Authorization: Bearer token1'
      config.http_post_header2 = 'Content-Type: application/json'
      config.http_post2_target = 'http://server2.com/backup'
      config.http_post2_header1 = 'X-API-Key: secret'
      config.http_post2_header2 = 'Custom-Header: value'
      config.http_get_target = 'http://server.com/settings'
      config.http_get_header1 = 'Accept: application/json'
      
      expect(config.http_post_target).toBe('http://server1.com/data')
      expect(config.http_post_header1).toBe('Authorization: Bearer token1')
      expect(config.http_post2_target).toBe('http://server2.com/backup')
      expect(config.http_get_target).toBe('http://server.com/settings')
    })
  })

  describe('InfluxDB configuration', () => {
    it('stores InfluxDB v2 settings', () => {
      const config = useConfigStore()
      
      config.influxdb2_target = 'http://influxdb.local:8086'
      config.influxdb2_bucket = 'brewery'
      config.influxdb2_org = 'myorg'
      config.influxdb2_token = 'secret-token-xyz'
      
      expect(config.influxdb2_target).toBe('http://influxdb.local:8086')
      expect(config.influxdb2_bucket).toBe('brewery')
      expect(config.influxdb2_org).toBe('myorg')
      expect(config.influxdb2_token).toBe('secret-token-xyz')
    })
  })

  describe('MQTT configuration', () => {
    it('stores MQTT server settings', () => {
      const config = useConfigStore()
      
      config.mqtt_target = 'mqtt.home.local'
      config.mqtt_port = 1883
      config.mqtt_user = 'homeassistant'
      config.mqtt_pass = 'mqtt-password'
      
      expect(config.mqtt_target).toBe('mqtt.home.local')
      expect(config.mqtt_port).toBe(1883)
      expect(config.mqtt_user).toBe('homeassistant')
      expect(config.mqtt_pass).toBe('mqtt-password')
    })
  })

  describe('PID controller configuration', () => {
    it('stores controller mode and target temperature', () => {
      const config = useConfigStore()

      config.controller_mode = 'f'
      config.target_temperature = 20.5

      expect(config.controller_mode).toBe('f')
      expect(config.target_temperature).toBe(20.5)
    })
  })

  describe('sendConfig with actual HTTP call', () => {
    it('calls postJson when config changes exist', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      http.postJson.mockResolvedValue({ success: true })

      const config = useConfigStore()
      // saveAll populates the configChanges baseline via saveConfigState
      vi.spyOn(config, 'sendConfig').mockResolvedValueOnce(true)
      await config.saveAll()
      vi.restoreAllMocks()

      // Now set a real baseline by spying on sendConfig and letting it run
      http.postJson.mockResolvedValue({ success: true })
      // Trigger saveConfigState by calling saveAll which calls sendConfig (true) then saveConfigState
      const sendConfigSpy = vi.spyOn(config, 'sendConfig').mockResolvedValueOnce(true)
      await config.saveAll()
      sendConfigSpy.mockRestore()

      // Mutate a field - now getConfigChanges will detect the difference
      config.mdns = '__changed__'

      const result = await config.sendConfig()
      expect(result).toBe(true)
      expect(http.postJson).toHaveBeenCalledWith('api/config', expect.objectContaining({ mdns: '__changed__' }))
    })

    it('returns false when postJson fails', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      http.postJson.mockResolvedValue({ success: true })

      const config = useConfigStore()
      // Populate baseline
      vi.spyOn(config, 'sendConfig').mockResolvedValueOnce(true)
      await config.saveAll()
      vi.restoreAllMocks()

      const sendConfigSpy2 = vi.spyOn(config, 'sendConfig').mockResolvedValueOnce(true)
      await config.saveAll()
      sendConfigSpy2.mockRestore()

      config.mdns = '__fail_test__'
      http.postJson.mockRejectedValue(new Error('Network error'))

      const result = await config.sendConfig()
      expect(result).toBe(false)
    })
  })

  describe('runWifiScan action', () => {
    it('returns success when wifi scan completes', async () => {
      const config = useConfigStore()
      vi.spyOn(config, 'sendWifiScan').mockResolvedValue(true)
      vi.spyOn(config, 'getWifiScanStatus').mockResolvedValue({
        success: true,
        data: { status: false, success: true, networks: ['net1'] }
      })

      const result = await config.runWifiScan()

      expect(result.success).toBe(true)
      expect(result.data.networks).toEqual(['net1'])
    })

    it('returns failure when sendWifiScan fails', async () => {
      const config = useConfigStore()
      vi.spyOn(config, 'sendWifiScan').mockResolvedValue(false)

      const result = await config.runWifiScan()

      expect(result.success).toBe(false)
    })

    it('returns failure when getWifiScanStatus fails', async () => {
      const config = useConfigStore()
      vi.spyOn(config, 'sendWifiScan').mockResolvedValue(true)
      vi.spyOn(config, 'getWifiScanStatus').mockResolvedValue({
        success: false,
        data: null
      })
      const global = mockGlobal

      const result = await config.runWifiScan()

      expect(result.success).toBe(false)
      expect(global.messageError).toContain('Failed to get wifi scan status')
    })

    it('polls again when scan is still in progress', async () => {
      vi.useFakeTimers()
      const config = useConfigStore()
      vi.spyOn(config, 'sendWifiScan').mockResolvedValue(true)

      let callCount = 0
      vi.spyOn(config, 'getWifiScanStatus').mockImplementation(async () => {
        callCount++
        if (callCount === 1) return { success: true, data: { status: true } }
        return { success: true, data: { status: false, success: true, networks: [] } }
      })

      const resultPromise = config.runWifiScan()
      await vi.runAllTimersAsync()
      const result = await resultPromise

      expect(result.success).toBe(true)
      expect(callCount).toBe(2)
      vi.useRealTimers()
    })

    it('sets error message when sendWifiScan fails inside runWifiScan', async () => {
      const config = useConfigStore()
      const global = mockGlobal
      vi.spyOn(config, 'sendWifiScan').mockResolvedValue(false)

      await config.runWifiScan()

      expect(global.messageError).toContain('Failed to start wifi scan')
    })
  })

  describe('runSensorScan action', () => {
    it('returns success when sensor scan completes', async () => {
      const config = useConfigStore()
      vi.spyOn(config, 'sendSensorScan').mockResolvedValue(true)
      vi.spyOn(config, 'getSensorScanStatus').mockResolvedValue({
        success: true,
        data: { status: false, success: true, sensors: ['sensor1'] }
      })

      const result = await config.runSensorScan()

      expect(result.success).toBe(true)
      expect(result.data.sensors).toEqual(['sensor1'])
    })

    it('returns failure when sendSensorScan fails', async () => {
      const config = useConfigStore()
      vi.spyOn(config, 'sendSensorScan').mockResolvedValue(false)

      const result = await config.runSensorScan()

      expect(result.success).toBe(false)
    })

    it('returns failure when getSensorScanStatus fails', async () => {
      const config = useConfigStore()
      vi.spyOn(config, 'sendSensorScan').mockResolvedValue(true)
      vi.spyOn(config, 'getSensorScanStatus').mockResolvedValue({
        success: false,
        data: null
      })

      const result = await config.runSensorScan()

      expect(result.success).toBe(false)
    })

    it('polls again when sensor scan is still in progress', async () => {
      vi.useFakeTimers()
      const config = useConfigStore()
      vi.spyOn(config, 'sendSensorScan').mockResolvedValue(true)

      let callCount = 0
      vi.spyOn(config, 'getSensorScanStatus').mockImplementation(async () => {
        callCount++
        if (callCount === 1) return { success: true, data: { status: true } }
        return { success: true, data: { status: false, success: true, sensors: [] } }
      })

      const resultPromise = config.runSensorScan()
      await vi.runAllTimersAsync()
      const result = await resultPromise

      expect(result.success).toBe(true)
      expect(callCount).toBe(2)
      vi.useRealTimers()
    })
  })

  describe('getWifiScanStatus error path', () => {
    it('returns failure on HTTP error', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      http.getJson.mockRejectedValue(new Error('Network error'))

      const config = useConfigStore()
      const result = await config.getWifiScanStatus()

      expect(result.success).toBe(false)
      expect(result.data).toBeNull()
    })
  })

  describe('getSensorScanStatus error path', () => {
    it('returns failure on HTTP error', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      http.getJson.mockRejectedValue(new Error('Network error'))

      const config = useConfigStore()
      const result = await config.getSensorScanStatus()

      expect(result.success).toBe(false)
      expect(result.data).toBeNull()
    })
  })

  describe('sendSensorScan error path', () => {
    it('returns false on HTTP error', async () => {
      const { sharedHttpClient: http } = await import('@mp-se/espframework-ui-components')
      http.request.mockRejectedValue(new Error('Request failed'))

      const config = useConfigStore()
      const result = await config.sendSensorScan()

      expect(result).toBe(false)
    })
  })

  describe('Bluetooth configuration', () => {
    it('stores BLE settings and sensor ID', () => {
      const config = useConfigStore()
      
      config.beer_ble_sensor_id = 'AA:BB:CC:DD:EE:FF'
      config.ble_push_enabled = true
      config.ble_scan_enabled = false
      config.ble_sensor_valid_time = 600
      
      expect(config.beer_ble_sensor_id).toBe('AA:BB:CC:DD:EE:FF')
      expect(config.ble_push_enabled).toBe(true)
      expect(config.ble_scan_enabled).toBe(false)
      expect(config.ble_sensor_valid_time).toBe(600)
    })
  })
})

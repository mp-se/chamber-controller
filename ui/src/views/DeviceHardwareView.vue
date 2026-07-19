<!--
  Chamber Controller UI
  Copyright (c) 2021-2026 Magnus

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
-->
<template>
  <div class="container">
    <p></p>
    <p class="h2">{{ t('device_hardware.title') }}</p>
    <hr />

    <form @submit.prevent="saveSettings" class="needs-validation" novalidate>
      <div class="row">
        <!-- Sensor Configuration -->
        <div class="col-md-6">
          <BsSelect
            v-model="config.fridge_sensor_id"
            :options="sensorOptions"
            :label="t('device_hardware.fridge_sensor_label')"
            :disabled="global.disabled"
          ></BsSelect>
        </div>
        <div class="col-md-6">
          <BsSelect
            v-model="config.beer_sensor_id"
            :options="sensorOptions"
            :label="t('device_hardware.beer_sensor_label')"
            :disabled="global.disabled"
          ></BsSelect>
        </div>

        <div class="col-md-6">
          <BsInputNumber
            v-model="config.fridge_sensor_offset"
            unit="°"
            :label="t('device_hardware.fridge_offset_label')"
            step="0.01"
            min="-5"
            max="5"
            width="4"
            :disabled="global.disabled"
          ></BsInputNumber>
        </div>
        <div class="col-md-6">
          <BsInputNumber
            v-model="config.beer_sensor_offset"
            unit="°"
            :label="t('device_hardware.beer_offset_label')"
            step="0.01"
            min="-5"
            max="5"
            width="4"
            :disabled="global.disabled"
          ></BsInputNumber>
        </div>

        <div class="col-md-12">
          <hr />
        </div>

        <!-- Relay Controls -->
        <div class="col-md-3">
          <BsInputSwitch
            v-model="config.enable_cooling"
            :label="t('device_hardware.enable_cooling')"
            :help="t('device_hardware.enable_cooling_help')"
            :disabled="global.disabled"
          ></BsInputSwitch>
        </div>
        <div class="col-md-3">
          <BsInputSwitch
            v-model="config.enable_heating"
            :label="t('device_hardware.enable_heating')"
            :help="t('device_hardware.enable_heating_help')"
            :disabled="global.disabled"
          ></BsInputSwitch>
        </div>
        <div class="col-md-3">
          <BsInputSwitch
            v-model="config.enable_fan"
            :label="t('device_hardware.enable_fan')"
            :help="t('device_hardware.enable_fan_help')"
            :disabled="global.disabled"
          ></BsInputSwitch>
        </div>
        <div class="col-md-3">
          <BsInputSwitch
            v-model="config.invert_pins"
            :label="t('device_hardware.invert_pins')"
            :help="t('device_hardware.invert_pins_help')"
            :disabled="global.disabled"
          ></BsInputSwitch>
        </div>

        <template v-if="global.feature.ble_sensor">
          <div class="col-md-12">
            <hr />
          </div>

          <div class="col-md-3">
            <BsInputSwitch
              v-model="config.ble_scan_enabled"
              :label="t('device_hardware.ble_scan')"
              :help="t('device_hardware.ble_scan_help')"
              width=""
              :disabled="global.disabled"
            ></BsInputSwitch>
          </div>

          <div class="col-md-3">
            <BsSelect
              v-model="config.ble_sensor_valid_time"
              :label="t('device_hardware.ble_valid_time')"
              :help="t('device_hardware.ble_valid_time_help')"
              :options="bleValidOptions"
              :disabled="global.disabled || !config.ble_scan_enabled"
            />
          </div>

          <div class="col-md-6">
            <BsSelect
              v-model="config.beer_ble_sensor_id"
              :label="t('device_hardware.ble_beer_sensor_label')"
              help="Select the beer BLE sensor, if you dont see your sensor, wait for it to be detected"
              :options="bleSensorOptions"
              :disabled="global.disabled || !config.ble_scan_enabled"
            />
          </div>
        </template>
      </div>

      <div class="row gy-2">
        <div class="col-md-12">
          <hr />
        </div>
        <div class="col-md-3">
          <button
            type="submit"
            class="btn btn-primary w-2"
            :disabled="global.disabled || !global.configChanged"
          >
            <span
              class="spinner-border spinner-border-sm"
              role="status"
              aria-hidden="true"
              v-show="global.disabled"
            ></span>
            &nbsp;{{ t('device_hardware.save') }}
          </button>
        </div>
      </div>
    </form>
  </div>
</template>

<script setup>
import { ref, onMounted } from 'vue'
import { useI18n } from 'vue-i18n'
import { validateCurrentForm, logDebug, logError } from '@mp-se/espframework-ui-components'
import { global, status, config } from '@/modules/pinia'

const { t } = useI18n()

const sensorOptions = ref([{ label: t('device_hardware.none'), value: '' }])
const bleSensorOptions = ref([{ label: t('device_hardware.none'), value: '' }])

const bleValidOptions = ref([
  { label: '5 minutes', value: 5 },
  { label: '10 minutes', value: 10 },
  { label: '15 minutes', value: 15 },
  { label: '20 minutes', value: 20 },
  { label: '25 minutes', value: 25 },
  { label: '30 minutes', value: 30 }
])

onMounted(() => {
  logDebug('DeviceHardwareView.onMounted()')
  global.disabled = true
  
  // Start both async operations in parallel to reduce mounting delay
  const p1 = status.load()
  const p2 = runSensorScan()
  
  Promise.all([p1, p2]).finally(() => {
    loadBleSensors()
    global.disabled = false
  })
})

const runSensorScan = async () => {
  try {
    const res = await config.runSensorScan()
    if (res.success && res.data && res.data.sensors) {
      sensorOptions.value = res.data.sensors.map((s) => ({
        label: s,
        value: s
      }))
      sensorOptions.value.unshift({ label: t('device_hardware.none'), value: '' })

      if (config.fridge_sensor_id && !sensorOptions.value.find((o) => o.value === config.fridge_sensor_id)) {
        sensorOptions.value.push({
          label: `${config.fridge_sensor_id} (not detected)`,
          value: config.fridge_sensor_id
        })
      }
      if (config.beer_sensor_id && !sensorOptions.value.find((o) => o.value === config.beer_sensor_id)) {
        sensorOptions.value.push({
          label: `${config.beer_sensor_id} (not detected)`,
          value: config.beer_sensor_id
        })
      }
    }
  } catch (err) {
    logError('DeviceHardwareView.runSensorScan()', err)
  }
}

const loadBleSensors = () => {
  if (status.temperature_device) {
    bleSensorOptions.value = status.temperature_device.map((s) => ({
      label: s.type ? `${s.device} (${s.type})` : s.device,
      value: s.device
    }))
  }
  bleSensorOptions.value.unshift({ label: t('device_hardware.none'), value: '' })

  if (
    config.beer_ble_sensor_id &&
    !bleSensorOptions.value.find((o) => o.value === config.beer_ble_sensor_id)
  ) {
    bleSensorOptions.value.push({
      label: `${config.beer_ble_sensor_id} (not detected)`,
      value: config.beer_ble_sensor_id
    })
  }
}

const saveSettings = async () => {
  if (!validateCurrentForm()) return
  await config.saveAll()
}

defineExpose({
  sensorOptions,
  bleSensorOptions,
  bleValidOptions,
  runSensorScan,
  loadBleSensors,
  saveSettings
})
</script>

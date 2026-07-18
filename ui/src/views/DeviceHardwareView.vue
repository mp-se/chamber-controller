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
    <p class="h2">Device - Settings</p>
    <hr />

    <form @submit.prevent="saveSettings" class="needs-validation" novalidate>
      <div class="row">
        <div class="col-md-6">
          <BsSelect
            v-model="config.fridge_sensor_id"
            label="Chamber Sensor"
            help="Select the chamber sensor"
            :options="sensorOptions"
            :disabled="global.disabled"
          />
        </div>

        <div class="col-md-6">
          <BsSelect
            v-model="config.beer_sensor_id"
            label="Beer Sensor"
            help="Select the beer sensor"
            :options="sensorOptions"
            :disabled="global.disabled"
          />
        </div>

        <div class="col-md-6">
          <BsInputNumber
            v-model="config.beer_sensor_offset"
            label="Beer Sensor Offset"
            help="Beer sensor offset value"
            min="-5"
            max="5"
            step="0.01"
            width="4"
            unit="°C"
            :disabled="global.disabled"
          />
        </div>

        <div class="col-md-6">
          <BsInputNumber
            v-model="config.fridge_sensor_offset"
            label="Fridge Sensor Offset"
            help="Fridge sensor offset value"
            min="-5"
            max="5"
            step="0.01"
            width="4"
            unit="°C"
            :disabled="global.disabled"
          />
        </div>

        <div class="col-md-12">
          <hr />
        </div>

        <div class="col-md-3">
          <BsInputSwitch
            v-model="config.enable_cooling"
            label="Enable Cooling"
            help="If cooling circuit is available"
            width=""
            :disabled="global.disabled"
          ></BsInputSwitch>
        </div>

        <div class="col-md-3">
          <BsInputSwitch
            v-model="config.enable_heating"
            label="Enable Heating"
            help="If heating circuit is available"
            width=""
            :disabled="global.disabled"
          ></BsInputSwitch>
        </div>

        <div class="col-md-3">
          <BsInputSwitch
            v-model="config.invert_pins"
            label="Invert pins"
            help="If pins should be inverted"
            width=""
            :disabled="global.disabled"
          ></BsInputSwitch>
        </div>

        <template v-if="global.feature.ble_sensor">
          <div class="col-md-12">
            <hr />
          </div>

          <div class="col-md-3" v-if="global.feature.ble_sensor">
            <BsInputSwitch
              v-model="config.ble_scan_enabled"
              label="Enable BLE sensors"
              help="Enable BLE scanning for temperature sensors"
              width=""
              :disabled="global.disabled"
            ></BsInputSwitch>
          </div>

          <div class="col-md-3">
            <BsSelect
              v-model="config.ble_sensor_valid_time"
              label="BLE sensor valid time"
              help="Select the valid time for the BLE sensor"
              :options="bleValidOptions"
              :disabled="global.disabled || !config.ble_scan_enabled"
            />
          </div>

          <div class="col-md-6">
            <BsSelect
              v-model="config.beer_ble_sensor_id"
              label="Beer BLE Sensor"
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
            &nbsp;Save
          </button>
        </div>
      </div>
    </form>
  </div>
</template>

<script setup>
import { ref, onMounted } from 'vue'
import { validateCurrentForm } from '@mp-se/espframework-ui-components'
import { global, config, status } from '@/modules/pinia'
import { logDebug } from '@mp-se/espframework-ui-components'

const sensorOptions = ref([{ label: '- not selected -', value: '' }])
const bleSensorOptions = ref([{ label: '- not selected -', value: '' }])
const bleValidOptions = ref([
  { label: '5 minutes', value: 5 },
  { label: '10 minutes', value: 10 },
  { label: '15 minutes', value: 15 },
  { label: '20 minutes', value: 20 },
  { label: '25 minutes', value: 25 },
  { label: '30 minutes', value: 30 }
])

onMounted(async () => {
  global.disabled = true

  // Populate the DS18B20 sensors
  const res = await config.runSensorScan()
  if (res && res.success && res.data) {
    const data = res.data
    logDebug('DeviceHardwareView::onMounted()', data)

    let fridge = false
    let beer = false

    for (const s of data.sensors) {
      if (s == config.beer_sensor_id) beer = true
      if (s == config.fridge_sensor_id) fridge = true

      sensorOptions.value.push({ label: s, value: s })
    }

    if (!beer && config.beer_sensor_id.length > 0)
      sensorOptions.value.push({
        label: config.beer_sensor_id + ' (not detected)',
        value: config.beer_sensor_id
      })

    if (!fridge && config.fridge_sensor_id.length > 0)
      sensorOptions.value.push({
        label: config.fridge_sensor_id + ' (not detected)',
        value: config.fridge_sensor_id
      })
  }

  // Populate the BLE sensors
  let ble = false
  await status.load()
  logDebug('DeviceHardwareView::onMounted() BLE sensor', status.temperature_device)

  for (const t of status.temperature_device) {
    bleSensorOptions.value.push({ label: t.device + ' (' + t.type + ')', value: t.device })
    if (t.device == config.beer_ble_sensor_id) ble = true
  }

  if (!ble && config.beer_ble_sensor_id.length > 0)
    bleSensorOptions.value.push({
      label: config.beer_ble_sensor_id + ' (not detected)',
      value: config.beer_ble_sensor_id
    })

  global.disabled = false
})

const saveSettings = async () => {
  if (!validateCurrentForm()) return

  await config.saveAll()
}
</script>

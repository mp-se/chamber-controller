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
  <div class="card h-100">
    <div class="card-header bg-success-subtle">Controller</div>
    <div class="card-body">
      <div class="row text-start">
        <div class="col-md-2">Mode:</div>
        <div class="col-md-10">
          <label class="h4">{{ mode }}</label>
        </div>
      </div>
      <div class="row text-start">
        <div class="col-md-2">State:</div>
        <div class="col-md-10">
          <label class="h4">{{ state }}</label>
        </div>
      </div>
      <div class="row text-start">
        <div class="col-md-2">Beer:</div>
        <div class="col-md-10">
          <label class="h4">{{ beerTemp }}</label>
        </div>
      </div>
      <div class="row text-start">
        <div class="col-md-2">Chamber:</div>
        <div class="col-md-10">
          <label class="h4">{{ fridgeTemp }}</label>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { computed } from 'vue'
import { status, config } from '@/modules/pinia'
import { formatTime } from '@mp-se/espframework-ui-components'

const beerTemp = computed(() => {
  // if (config.beer_sensor_id == '') return '--' + '°' + config.temp_format
  if (!status.pid_beer_temp_connected) return '--' + '°' + config.temp_format
  return status.pid_beer_temp + '°' + config.temp_format
})

const fridgeTemp = computed(() => {
  // if (config.fridge_sensor_id == '') return '--' + '°' + config.temp_format
  if (!status.pid_fridge_temp_connected) return '--' + '°' + config.temp_format
  return status.pid_fridge_temp + '°' + config.temp_format
})

const state = computed(() => {
  switch (status.pid_state) {
    case 0: // IDLE, min time since heating or cooling
      return (
        'Idle for ' +
        formatTime(
          status.pid_time_since_cooling < status.pid_time_since_heating
            ? status.pid_time_since_cooling
            : status.pid_time_since_heating
        )
      )

    case 1: // OFF
      return 'Off'

    case 2: // HEATING
      return 'Heating for ' + formatTime(status.pid_time_since_idle)

    case 3: // COOLING
      return 'Cooling for ' + formatTime(status.pid_time_since_idle)

    case 4: // WAITING TO COOL
      return 'Waiting to cool ' + formatTime(status.pid_wait_time)

    case 5: // WAITING TO HEAT
      return 'Waiting to heat ' + formatTime(status.pid_wait_time)

    case 6: // WAITING FOR PEEK DETECT
      return 'Waiting for peek detect ' + formatTime(status.pid_wait_time)

    case 7: // COOLING MIN TIME
      return 'Cooling min time ' + formatTime(status.pid_time_since_idle)

    case 8: // HEATING MIN TIME
      return 'Heating min time ' + formatTime(status.pid_time_since_idle)
  }

  return 'Unknown status...'
})

const mode = computed(() => {
  if (status.pid_mode == 'f')
    return 'Chamber constant, target ' + status.pid_fridge_target_temp + '°' + config.temp_format
  if (status.pid_mode == 'b')
    return 'Beer constant, target ' + status.pid_beer_target_temp + '°' + config.temp_format

  return 'Off'
})
</script>

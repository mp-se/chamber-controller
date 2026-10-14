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
    <p class="h3">{{ t('brewgraph.title') }}</p>
    <hr />

    <form @submit.prevent="save" class="needs-validation" novalidate>
      <div class="row">
        <div class="col-md-12">
          <BsInputText
            v-model="config.brewgraph_url"
            type="url"
            maxlength="200"
            :label="t('brewgraph.url_label')"
            :help="t('brewgraph.url_help')"
            :disabled="global.disabled"
          />
        </div>
        <div class="col-md-12">
          <BsInputText
            v-model="config.brewgraph_token"
            type="password"
            maxlength="128"
            :label="t('brewgraph.token_label')"
            :help="t('brewgraph.token_help')"
            :disabled="global.disabled"
          />
        </div>
        <div class="col-md-6">
          <BsSelect
            v-model="config.brewgraph_interval"
            :options="intervalOptions"
            :label="t('brewgraph.interval_label')"
            :help="t('brewgraph.interval_help')"
            :disabled="global.disabled"
          />
        </div>
        <div class="col-md-12">
          <BsInputSwitch
            v-model="config.brewgraph_remote_control_enabled"
            :label="t('brewgraph.remote_control_label')"
            :help="t('brewgraph.remote_control_help')"
            :disabled="global.disabled"
          />
        </div>
      </div>
      <div class="row gy-2">
        <div class="col-md-12">
          <hr />
        </div>
        <div class="col-sm-3">
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
            &nbsp;{{ t('brewgraph.save') }}
          </button>
        </div>
      </div>
    </form>
  </div>
</template>

<script setup>
import { validateCurrentForm } from '@mp-se/espframework-ui-components'
import { global, config } from '@/modules/pinia'
import { useI18n } from 'vue-i18n'

const { t } = useI18n()

// The server accepts one request per 5 minutes, these are the choices
const intervalOptions = [5, 10, 15, 20].map((n) => ({
  label: t('brewgraph.interval_minutes', { n }),
  value: n
}))

const save = async () => {
  if (!validateCurrentForm()) return

  await config.saveAll()
}

defineExpose({
  intervalOptions,
  save
})
</script>

import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import PidDataFragment from '../PidDataFragment.vue'
import { flushPromises } from '@vue/test-utils'
import { global } from '@/modules/pinia'
import { sharedHttpClient as http } from '@mp-se/espframework-ui-components'

describe('PidDataFragment', () => {
  beforeEach(() => {
    vi.clearAllMocks()
    global.disabled = false
    global.messageError = ''
    global.clearMessages = vi.fn()
    http.getJson.mockResolvedValue({ value: 'test data' })
  })

  const createWrapper = (source = 'cc') =>
    mount(PidDataFragment, {
      props: { source }
    })

  describe('component mounting', () => {
    it('mounts without error', async () => {
      const wrapper = createWrapper()
      await flushPromises()
      expect(wrapper.exists()).toBe(true)
    })

    it('displays Reload button', async () => {
      const wrapper = createWrapper()
      await flushPromises()
      expect(wrapper.find('button').exists()).toBe(true)
      expect(wrapper.text()).toContain('Reload')
    })

    it('calls load on mount', async () => {
      createWrapper('cc')
      await flushPromises()
      expect(http.getJson).toHaveBeenCalledWith(expect.stringContaining('api/pid/cc'))
    })
  })

  describe('dataSource computed', () => {
    it('shows Control Constants for cc source', async () => {
      const wrapper = createWrapper('cc')
      await flushPromises()
      expect(wrapper.text()).toContain('Control Constants')
    })

    it('shows Control Variables for cv source', async () => {
      const wrapper = createWrapper('cv')
      await flushPromises()
      expect(wrapper.text()).toContain('Control Variables')
    })

    it('shows Min Times for mt source', async () => {
      const wrapper = createWrapper('mt')
      await flushPromises()
      expect(wrapper.text()).toContain('Min Times')
    })

    it('shows Control Settings for cs source', async () => {
      const wrapper = createWrapper('cs')
      await flushPromises()
      expect(wrapper.text()).toContain('Control Settings')
    })

    it('shows Unknown for unrecognized source', async () => {
      const wrapper = createWrapper('xx')
      await flushPromises()
      expect(wrapper.text()).toContain('Unknown')
    })
  })

  describe('Reload button click', () => {
    it('clicking reload button triggers load again', async () => {
      http.getJson.mockResolvedValue({ updated: true })
      const wrapper = createWrapper('cc')
      await flushPromises()
      http.getJson.mockClear()

      const button = wrapper.find('button')
      await button.trigger('click')
      await flushPromises()

      expect(http.getJson).toHaveBeenCalled()
    })
  })

  describe('load error handling', () => {
    it('sets error message when load fails', async () => {
      http.getJson.mockRejectedValue(new Error('Network error'))
      global.messageError = ''
      createWrapper('cc')
      await flushPromises()
      expect(global.messageError).toContain('Failed to do load data')
    })

    it('re-enables controls after load error', async () => {
      http.getJson.mockRejectedValue(new Error('Network error'))
      global.disabled = false
      createWrapper('cc')
      await flushPromises()
      expect(global.disabled).toBe(false)
    })

    it('handles reload button click when load fails', async () => {
      http.getJson.mockResolvedValue({ data: 'ok' })
      const wrapper = createWrapper('cc')
      await flushPromises()

      http.getJson.mockRejectedValue(new Error('Reload failed'))
      const button = wrapper.find('button')
      await button.trigger('click')
      await flushPromises()

      expect(global.messageError).toContain('Failed to do load data')
    })
  })

  describe('state management', () => {
    it('disables reload button when global.disabled is true', async () => {
      http.getJson.mockResolvedValue({ data: 'ok' })
      const wrapper = createWrapper('cc')
      await flushPromises()
      // Set disabled after load completes (load() resets disabled to false when done)
      global.disabled = true
      await wrapper.vm.$nextTick()
      const button = wrapper.find('button')
      expect(button.element.disabled).toBe(true)
    })
  })
})

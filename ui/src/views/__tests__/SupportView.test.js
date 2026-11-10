import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import SupportView from '../SupportView.vue'
import { createTestingPinia } from '../../tests/testUtils'

describe('SupportView', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('mounts without error', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(SupportView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsMessage: true
        }
      }
    })
    expect(wrapper.exists()).toBe(true)
  })

  it('shows the hardware test result as a json document', async () => {
    const pinia = createTestingPinia()
    const wrapper = mount(SupportView, {
      global: { plugins: [pinia], stubs: { BsMessage: true } }
    })
    const { config } = await import('@/modules/pinia')
    config.runHardwareScan = vi.fn(async () => ({
      success: true,
      data: {
        chip: { family: 'ESP32S3', cores: 2, revision: 0, features: [] },
        onewire: {
          pin: 13,
          count: 2,
          sensors: [
            { address: '28FF000000000001', valid: true, temp: 4.5, role: 'fridge' },
            { address: '28FF000000000002', valid: false, role: '' }
          ],
          configured: {
            fridge: { id: '28FF000000000001', detected: true },
            beer: { id: '28FF0000000000AA', detected: false }
          }
        }
      }
    }))
    await wrapper
      .findAll('button')
      .find((b) => b.text().includes('Hardware test'))
      .trigger('click')
    await vi.waitFor(() => expect(wrapper.find('pre').text()).toContain('28FF000000000002'))
    const doc = JSON.parse(wrapper.find('pre').text())
    expect(doc.onewire.sensors[0].temp).toBe(4.5)
    expect(doc.onewire.configured.beer.detected).toBe(false)
  })

  it('displays support information', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(SupportView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsMessage: { template: '<div><slot /></div>' }
        }
      }
    })
    expect(wrapper.text().length).toBeGreaterThan(0)
  })
})

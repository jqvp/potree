
import * as THREE from "../../../libs/three.js/build/three.module.js";
import { PointCloudOctreeGeometryNode } from "../../PointCloudOctreeGeometry.js";
import {Utils} from "../../utils.js";
import { BoxVolume, SphereVolume} from "../../utils/Volume.js";

import {MeasurePanel} from "./MeasurePanel.js";

export class VolumePanel extends MeasurePanel{
	constructor(viewer, measurement, propertiesPanel){
		super(viewer, measurement, propertiesPanel);

		let copyIconPath = Potree.resourcePath + '/icons/copy.svg';
		let removeIconPath = Potree.resourcePath + '/icons/remove.svg';

		let lblLengthText = new Map([
			[BoxVolume, "length"],
			[SphereVolume, "rx"],
		]).get(measurement.constructor);

		let lblWidthText = new Map([
			[BoxVolume, "width"],
			[SphereVolume, "ry"],
		]).get(measurement.constructor);

		let lblHeightText = new Map([
			[BoxVolume, "height"],
			[SphereVolume, "rz"],
		]).get(measurement.constructor);

		this.elContent = $(`
			<div class="measurement_content selectable">
				<span class="coordinates_table_container"></span>

				<table class="measurement_value_table">
					<tr>
						<th>\u03b1</th>
						<th>\u03b2</th>
						<th>\u03b3</th>
						<th></th>
					</tr>
					<tr>
						<td align="center" id="angle_cell_alpha" style="width: 33%"></td>
						<td align="center" id="angle_cell_betta" style="width: 33%"></td>
						<td align="center" id="angle_cell_gamma" style="width: 33%"></td>
						<td align="right" style="width: 25%">
							<img name="copyRotation" title="copy" class="button-icon" src="${copyIconPath}" style="width: 16px; height: 16px"/>
						</td>
					</tr>
				</table>

				<table class="measurement_value_table">
					<tr>
						<th>${lblLengthText}</th>
						<th>${lblWidthText}</th>
						<th>${lblHeightText}</th>
						<th></th>
					</tr>
					<tr>
						<td align="center" id="cell_length" style="width: 33%"></td>
						<td align="center" id="cell_width" style="width: 33%"></td>
						<td align="center" id="cell_height" style="width: 33%"></td>
						<td align="right" style="width: 25%">
							<img name="copyScale" title="copy" class="button-icon" src="${copyIconPath}" style="width: 16px; height: 16px"/>
						</td>
					</tr>
				</table>

				<br>
				<span style="font-weight: bold">Volume: </span>
				<span id="measurement_volume"></span>

				<!--
				<li>
					<label style="whitespace: nowrap">
						<input id="volume_show" type="checkbox"/>
						<span>show volume</span>
					</label>
				</li>-->

				<li>
					<label style="whitespace: nowrap">
						<input id="volume_clip" type="checkbox"/>
						<span>make clip volume</span>
					</label>
				</li>

				<li style="margin-top: 10px">
					<input name="download_volume" type="button" value="prepare download" style="width: 100%" />
					<div name="download_message"></div>
				</li>


				<!-- ACTIONS -->
				<li style="display: grid; grid-template-columns: auto auto; grid-column-gap: 5px; margin-top: 10px">
					<input id="volume_reset_orientation" type="button" value="reset orientation"/>
					<input id="volume_make_uniform" type="button" value="make uniform"/>
				</li>
				<div style="display: flex; margin-top: 12px">
					<span></span>
					<span style="flex-grow: 1"></span>
					<img name="remove" class="button-icon" src="${removeIconPath}" style="width: 16px; height: 16px"/>
				</div>
			</div>
		`);

		{ // download
			this.elDownloadButton = this.elContent.find("input[name=download_volume]");

			if(this.propertiesPanel.viewer.server){
				this.elDownloadButton.click(() => this.download());
			} else {
				this.elDownloadButton.hide();
			}
		}

		this.elCopyRotation = this.elContent.find("img[name=copyRotation]");
		this.elCopyRotation.click( () => {
			let rotation = this.measurement.rotation.toArray().slice(0, 3);
			let msg = rotation.map(c => c.toFixed(3)).join(", ");
			Utils.clipboardCopy(msg);

			this.viewer.postMessage(
					`Copied value to clipboard: <br>'${msg}'`,
					{duration: 3000});
		});

		this.elCopyScale = this.elContent.find("img[name=copyScale]");
		this.elCopyScale.click( () => {
			let scale = this.measurement.scale.toArray();
			let msg = scale.map(c => c.toFixed(3)).join(", ");
			Utils.clipboardCopy(msg);

			this.viewer.postMessage(
					`Copied value to clipboard: <br>'${msg}'`,
					{duration: 3000});
		});

		if (this.measurement instanceof BoxVolume) {
			this.requests = [];
			this.scheduledRecomputeTime = null;
			this.measurement.numPoints = 0;
			this.filter = -1;
			this.toClassNumber = 0;

			if (this.viewer.scene.pointclouds[0]) {
				let elVolume = this.elContent.find("#measurement_volume");
				elVolume.after(`
					<br>
					<span style="font-weight: bold">Puntos: </span>
					<span id="num_points_volume">-</span>
				`);
				this.elPoints = this.elContent.find("#num_points_volume");
				this.elPoints.after(`
					<div class="divider">
						<span>Cambio de clasificación</span>
					</div>
					<span style="font-weight: bold">Filtro:</span>
					<li>
						<select id="optClassFilter" name="optClassFilter"></select>
					</li>
					<span style="font-weight: bold">Cambiar a:</span>
					<li>
						<select id="optToClass" name="optToClass"></select>
					</li>
				`);

				let filterSelect = this.elContent.find('#optClassFilter');
				let toClassSelect = this.elContent.find('#optToClass');
				const classifications = this.viewer.scene.pointclouds[0].material.classification;
				const options = Object.keys(classifications);

				options.forEach(a => {
					let elOption = $(`<option value=${a}>${classifications[a].name}</option>`);
					filterSelect.append($(`<option value=${a}>${classifications[a].name}</option>`));
					toClassSelect.append(elOption);
				});
				filterSelect.append($(`<option value=-1>No filter</option>`));
				filterSelect.val("No filter");
				toClassSelect.val("");

				filterSelect.selectmenu({change: (event, ui) => {
					console.log(event, ui);

					this.filter = parseInt(ui.item.value);
				}});
				toClassSelect.selectmenu({change: (event, ui) => {
					this.toClassNumber = parseInt(ui.item.value);
				}});
			}

			this.elContent.find("#volume_reset_orientation").parent().after(`
				<li style="display: grid; grid-template-columns: auto auto; grid-column-gap: 5px; margin-top: 10px">
					<input id="volume_get_points" type="button" value="Remplazar clasificación"/>
					<input id="volume_cancel_get" type="button" value="Cancelar proceso"/>
				</li>
			`);

			this.elContent.find("#volume_get_points").click(() => {
				this.resetRequests();
			});
	
			this.elContent.find("#volume_cancel_get").click(() => {
				this.cancelRequests();
			});
		}

		this.elRemove = this.elContent.find("img[name=remove]");
		this.elRemove.click( () => {
			this.viewer.scene.removeVolume(measurement);
		});

		this.elContent.find("#volume_reset_orientation").click(() => {
			measurement.rotation.set(0, 0, 0);
		});

		this.elContent.find("#volume_make_uniform").click(() => {
			let mean = (measurement.scale.x + measurement.scale.y + measurement.scale.z) / 3;
			measurement.scale.set(mean, mean, mean);
		});

		this.elCheckClip = this.elContent.find('#volume_clip');
		this.elCheckClip.click(event => {
			this.measurement.clip = event.target.checked;
		});

		this.elCheckShow = this.elContent.find('#volume_show');
		this.elCheckShow.click(event => {
			this.measurement.visible = event.target.checked;
		});

		this.propertiesPanel.addVolatileListener(measurement, "position_changed", () => this._update);
		this.propertiesPanel.addVolatileListener(measurement, "orientation_changed", () => this._update);
		this.propertiesPanel.addVolatileListener(measurement, "scale_changed", () => this._update);
		this.propertiesPanel.addVolatileListener(measurement, "clip_changed", this._update);

		this.update();
	}

	async download(){

		let clipBox = this.measurement;

		let regions = [];
		//for(let clipBox of boxes){
		{
			let toClip = clipBox.matrixWorld;

			let px = new THREE.Vector3(+0.5, 0, 0).applyMatrix4(toClip);
			let nx = new THREE.Vector3(-0.5, 0, 0).applyMatrix4(toClip);
			let py = new THREE.Vector3(0, +0.5, 0).applyMatrix4(toClip);
			let ny = new THREE.Vector3(0, -0.5, 0).applyMatrix4(toClip);
			let pz = new THREE.Vector3(0, 0, +0.5).applyMatrix4(toClip);
			let nz = new THREE.Vector3(0, 0, -0.5).applyMatrix4(toClip);

			let pxN = new THREE.Vector3().subVectors(nx, px).normalize();
			let nxN = pxN.clone().multiplyScalar(-1);
			let pyN = new THREE.Vector3().subVectors(ny, py).normalize();
			let nyN = pyN.clone().multiplyScalar(-1);
			let pzN = new THREE.Vector3().subVectors(nz, pz).normalize();
			let nzN = pzN.clone().multiplyScalar(-1);

			let planes = [
				new THREE.Plane().setFromNormalAndCoplanarPoint(pxN, px),
				new THREE.Plane().setFromNormalAndCoplanarPoint(nxN, nx),
				new THREE.Plane().setFromNormalAndCoplanarPoint(pyN, py),
				new THREE.Plane().setFromNormalAndCoplanarPoint(nyN, ny),
				new THREE.Plane().setFromNormalAndCoplanarPoint(pzN, pz),
				new THREE.Plane().setFromNormalAndCoplanarPoint(nzN, nz),
			];

			let planeQueryParts = [];
			for(let plane of planes){
				let part = [plane.normal.toArray(), plane.constant].join(",");
				part = `[${part}]`;
				planeQueryParts.push(part);
			}
			let region = "[" + planeQueryParts.join(",") + "]";
			regions.push(region);
		}

		let regionsArg = regions.join(",");

		let pointcloudArgs = [];
		for(let pointcloud of this.viewer.scene.pointclouds){
			if(!pointcloud.visible){
				continue;
			}

			let offset = pointcloud.pcoGeometry.offset.clone();
			let negateOffset = new THREE.Matrix4().makeTranslation(...offset.multiplyScalar(-1).toArray());
			let matrixWorld = pointcloud.matrixWorld;

			let transform = new THREE.Matrix4().multiplyMatrices(matrixWorld, negateOffset);

			let path = `${window.location.pathname}/../${pointcloud.pcoGeometry.url}`;

			let arg = {
				path: path,
				transform: transform.elements,
			};
			let argString = JSON.stringify(arg);

			pointcloudArgs.push(argString);
		}
		let pointcloudsArg = pointcloudArgs.join(",");

		let elMessage = this.elContent.find("div[name=download_message]");

		let error = (message) => {
			elMessage.html(`<div style="color: #ff0000">ERROR: ${message}</div>`);
		};

		let info = (message) => {
			elMessage.html(`${message}`);
		};

		let handle = null;
		{ // START FILTER
			let url = `${viewer.server}/create_regions_filter?pointclouds=[${pointcloudsArg}]&regions=[${regionsArg}]`;
			
			//console.log(url);

			info("estimating results ...");

			let response = await fetch(url);
			let jsResponse = await response.json();
			//console.log(jsResponse);

			if(!jsResponse.handle){
				error(jsResponse.message);
				return;
			}else{
				handle = jsResponse.handle;
			}
		}

		{ // WAIT, CHECK PROGRESS, HANDLE FINISH
			let url = `${viewer.server}/check_regions_filter?handle=${handle}`;

			let sleep = (function(duration){
				return new Promise( (res, rej) => {
					setTimeout(() => {
						res();
					}, duration);
				});
			});

			let handleFiltering = (jsResponse) => {
				let {progress, estimate} = jsResponse;

				let progressFract = progress["processed points"] / estimate.points;
				let progressPercents = parseInt(progressFract * 100);

				info(`progress: ${progressPercents}%`);
			};

			let handleFinish = (jsResponse) => {
				let message = "downloads ready: <br>";
				message += "<ul>";

				for(let i = 0; i < jsResponse.pointclouds.length; i++){
					let url = `${viewer.server}/download_regions_filter_result?handle=${handle}&index=${i}`;

					message += `<li><a href="${url}">result_${i}.las</a> </li>\n`;
				}

				let reportURL = `${viewer.server}/download_regions_filter_report?handle=${handle}`;
				message += `<li> <a href="${reportURL}">report.json</a> </li>\n`;
				message += "</ul>";

				info(message);
			};

			let handleUnexpected = (jsResponse) => {
				let message = `Unexpected Response. <br>status: ${jsResponse.status} <br>message: ${jsResponse.message}`;
				info(message);
			};

			let handleError = (jsResponse) => {
				let message = `ERROR: ${jsResponse.message}`;
				error(message);

				throw new Error(message);
			};

			let start = Date.now();

			while(true){
				let response = await fetch(url);
				let jsResponse = await response.json();

				if(jsResponse.status === "ERROR"){
					handleError(jsResponse);
				}else if(jsResponse.status === "FILTERING"){
					handleFiltering(jsResponse);
				}else if(jsResponse.status === "FINISHED"){
					handleFinish(jsResponse);

					break;
				}else{
					handleUnexpected(jsResponse);
				}

				let durationS = (Date.now() - start) / 1000;
				let sleepAmountMS = durationS < 10 ? 100 : 1000;

				await sleep(sleepAmountMS);
			}
		}

	}

	update(){
		let elCoordiantesContainer = this.elContent.find('.coordinates_table_container');
		elCoordiantesContainer.empty();
		elCoordiantesContainer.append(this.createCoordinatesTable([this.measurement.position]));

		{
			let angles = this.measurement.rotation.toVector3();
			angles = angles.toArray();
			//angles = [angles.z, angles.x, angles.y];
			angles = angles.map(v => 180 * v / Math.PI);
			angles = angles.map(a => a.toFixed(1) + '\u00B0');

			let elAlpha = this.elContent.find(`#angle_cell_alpha`);
			let elBetta = this.elContent.find(`#angle_cell_betta`);
			let elGamma = this.elContent.find(`#angle_cell_gamma`);

			elAlpha.html(angles[0]);
			elBetta.html(angles[1]);
			elGamma.html(angles[2]);
		}

		{
			let dimensions = this.measurement.scale.toArray();
			dimensions = dimensions.map(v => Utils.addCommas(v.toFixed(2)));

			let elLength = this.elContent.find(`#cell_length`);
			let elWidth = this.elContent.find(`#cell_width`);
			let elHeight = this.elContent.find(`#cell_height`);

			elLength.html(dimensions[0]);
			elWidth.html(dimensions[1]);
			elHeight.html(dimensions[2]);
		}

		{
			let elVolume = this.elContent.find(`#measurement_volume`);
			let volume = this.measurement.getVolume();
			elVolume.html(Utils.addCommas(volume.toFixed(2)));
		}

		if (this.measurement instanceof BoxVolume) {
			this.elPoints.text(this.measurement.numPoints.toString());
		}

		this.elCheckClip.prop("checked", this.measurement.clip);
		this.elCheckShow.prop("checked", this.measurement.visible);

	}

	progressHandler (pointcloud, event) {
		this.measurement.numPoints += event.numPoints;
		this.elPoints.text(this.measurement.numPoints.toString());
	}

	resetRequests () {		
		this.cancelRequests();
		this.measurement.numPoints = 0;

		if ((this.scheduledRecomputeTime || 0) > new Date().getTime()) {
			return;
		} else {
			this.scheduledRecomputeTime = new Date().getTime() + 100;
		}

		for (let pointcloud of this.viewer.scene.pointclouds.filter(p => p.visible)) {
			this.requests.push(pointcloud.getPointsInVolume(this.measurement, null, {
				'onProgress': (event) => {
					this.progressHandler(pointcloud, event.points);
				},
				'onFinish': (event) => {
				},
				'onCancel': () => {
				},
				/**
				 * @param {PointCloudOctreeGeometryNode} node
				 * @param {Uint32Array} acceptedIndices
				 * @param {Float32Array} acceptedXYZ
				 */
				'onAcceptedPoints': (node, acceptedIndices, acceptedXYZ) => {
					console.log(node, acceptedIndices);
					let arrClass = node.geometry.attributes['classification'].array;
					const filteredIndices = this.filter === -1 ? acceptedIndices
						: acceptedIndices.filter((v) => arrClass[v] === this.filter);

					for (const i of filteredIndices) arrClass[i] = this.toClassNumber;
					node.geometry.attributes['classification'].needsUpdate = true;
					
					// THE FOLLOWING ONLY WORKS WITHOUT COMPRESSION
					if (node.octreeGeometry.loader.metadata.encoding === "BROTLI") {
						throw new Error("The pointcloud patching functionality only works for uncompressed pointclouds.");
					}

					fetch(node.octreeGeometry.url.slice(0, -"metadata.json".length-1), {
						headers: {
							'Content-Type': 'application/json',
						},
						method: "PATCH",
						body: JSON.stringify({
							indices: Array.from(filteredIndices),
							classification: this.toClassNumber,
							byteOffset: node.byteOffset.toString()
						}),
					}).then(res => {
						console.log(res);
					}).catch(err => {
						console.error(err);
					});
				}
			}));
		}
	}

	cancelRequests () {
		for (let request of this.requests) {
			request.cancel();
		}
		
		this.requests = [];
	}
};
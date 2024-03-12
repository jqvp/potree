
import * as THREE from "../../libs/three.js/build/three.module.js";
import {Points} from "../Points.js";
import { BoxVolume } from "./Volume.js";

/**
 * Progressively gets points from a pointcloud according to its intersection  
 * with a shape (box) and collects them on a THREE object.
 * 
 * It progressively gives points through the callback functions
 */
export class PointPickerRequest {
    /**
     * 
     * @param {PointCloudOctree} pointcloud The pointcloud used to get the
     * points from
     * @param {BoxVolume} shape The shape that will be used to calculate the intersection
     * @param {Number} maxDepth Maximum depth the object will search in the 
     * pointcloud
     * @param {{onProgress: ({}) => void, onFinish: ({}) => void, onCancel: ({}) => void}
     * } callback Object containing callback functions
     */
	constructor (pointcloud, shape, maxDepth, callback) {
		this.pointcloud = pointcloud;
		this.shape = shape;
		this.maxDepth = maxDepth || Number.MAX_VALUE;
		this.callback = callback;
		/**
		 * @type {{size: Number, points: Points}}
		 */
		this.temporaryResult = {size: 0, points: new Points()};
		this.pointsServed = 0;
		this.highestLevelServed = 0;

		this.priorityQueue = new BinaryHeap(function (x) { return 1 / x.weight; });

		this.initialize();
	}

	initialize () {
		this.priorityQueue.push({node: this.pointcloud.pcoGeometry.root, weight: Infinity});
	};

	// traverse the node and add intersecting descendants to queue
	traverse (node) {
		let stack = [];
		for (let i = 0; i < 8; i++) {
			let child = node.children[i];
			if (child && this.doesIntersect(this.shape, child)) {
				stack.push(child);
			}
		}

		while (stack.length > 0) {
			let node = stack.pop();
			let weight = node.boundingSphere.radius;

			this.priorityQueue.push({node: node, weight: weight});

			// add children that intersect the cutting plane
			if (node.level < this.maxDepth) {
				for (let i = 0; i < 8; i++) {
					let child = node.children[i];
					if (child && this.doesIntersect(this.shape, child)) {
						stack.push(child);
					}
				}
			}
		}
	}

	update(){
		if(!this.updateGeneratorInstance){
			this.updateGeneratorInstance = this.updateGenerator();
		}

		let result = this.updateGeneratorInstance.next();
		if(result.done){
			this.updateGeneratorInstance = null;
		}
	}

	* updateGenerator(){
		// load nodes in queue
		// if hierarchy expands, also load nodes from expanded hierarchy
		// once loaded, add data to this.points and remove node from queue
		// only evaluate 1-50 nodes per frame to maintain responsiveness

		let start = performance.now();

		let maxNodesPerUpdate = 25;
		let intersectedNodes = [];

		for (let i = 0; i < Math.min(maxNodesPerUpdate, this.priorityQueue.size()); i++) {
			let element = this.priorityQueue.pop();
			let node = element.node;

			if(node.level > this.maxDepth){
				continue;
			}

			if (node.loaded) {
				// add points to result
				intersectedNodes.push(node);
				exports.lru.touch(node);
				this.highestLevelServed = Math.max(node.getLevel(), this.highestLevelServed);

				var geom = node.pcoGeometry;
				var hierarchyStepSize = geom ? geom.hierarchyStepSize : 1;

				var doTraverse = node.getLevel() === 0 ||
					(node.level % hierarchyStepSize === 0 && node.hasChildren);

				if (doTraverse) {
					this.traverse(node);
				}
			} else {
				node.load();
				this.priorityQueue.push(element);
			}
		}

		if (intersectedNodes.length > 0) {

			for(let done of this.getPointsInsideShape(intersectedNodes, this.temporaryResult)){
				if(!done){
					yield false;
				}
			}
			if (this.temporaryResult.size > 100) {//TODO
				this.pointsServed += this.temporaryResult.size;
				this.callback.onProgress({request: this, points: this.temporaryResult});
				this.temporaryResult = {size: 0, points: new Points()};
			}
		}

		if (this.priorityQueue.size === 0) {//TODO
			// we're done! inform callback and remove from pending requests

			if (this.temporaryResult.size > 0) {
				this.pointsServed += this.temporaryResult.size;
				this.callback.onProgress({request: this, points: this.temporaryResult});
				this.temporaryResult = {size: 0, points: new Points()};
			}

			this.callback.onFinish({request: this});

			let index = this.pointcloud.pointPickerRequests.indexOf(this);
			if (index >= 0) {
				this.pointcloud.pointPickerRequests.splice(index, 1);
			}
		}

		yield true;
	};

	* getAccepted(numPoints, node, matrix, points){
		let checkpoint = performance.now();

		let accepted = new Uint32Array(numPoints);
		let acceptedPositions = new Float32Array(numPoints * 3);
		let numAccepted = 0;

		let pos = new THREE.Vector3();
		const center = new THREE.Vector3().applyMatrix4(this.shape.matrixWorld);

		let view = new Float32Array(node.geometry.attributes.position.array);

		for (let i = 0; i < numPoints; i++) {

			pos.set(
				view[i * 3 + 0],
				view[i * 3 + 1],
				view[i * 3 + 2]);

			pos.applyMatrix4(matrix);
			let diff = new THREE.Vector3().subVectors(pos, center)
			.applyQuaternion(this.shape.quaternion.conjugate());

			if (Math.abs(diff.x) < this.shape.scale.x/2
			 && Math.abs(diff.y) < this.shape.scale.y/2
			 && Math.abs(diff.z) < this.shape.scale.z/2) {
				
				accepted[numAccepted] = i;
				points.boundingBox.expandByPoint(pos);

				pos.sub(this.pointcloud.position);

				acceptedPositions[3 * numAccepted + 0] = pos.x;
				acceptedPositions[3 * numAccepted + 1] = pos.y;
				acceptedPositions[3 * numAccepted + 2] = pos.z;

				numAccepted++;
			}

			if((i % 1000) === 0){
				let duration = performance.now() - checkpoint;
				if(duration > 4){
					yield false;
					checkpoint = performance.now();
				}
			}
		}

		accepted = accepted.subarray(0, numAccepted);
		acceptedPositions = acceptedPositions.subarray(0, numAccepted * 3);

		yield [accepted, acceptedPositions];
	}

	/**
	 * 
	 * @param {PointCloudOctreeGeometryNode} nodes 
	 * @param {{size: Number, points: Points}} target 
	 */
	* getPointsInsideShape(nodes, target){
		let checkpoint = performance.now();

		let pointsProcessed = 0;

		for (let node of nodes) {
			let numPoints = node.numPoints;
			let geometry = node.geometry;

			// skip if current node doesn't intersect current segment
			if(!numPoints || !this.doesIntersect(this.shape, node)){
				continue;
			}

			let points = new Points();

			let nodeMatrix = new THREE.Matrix4().makeTranslation(...node.boundingBox.min.toArray());

			let matrix = new THREE.Matrix4()
				.multiplyMatrices(this.pointcloud.matrixWorld, nodeMatrix);

			pointsProcessed = pointsProcessed + numPoints;

			let accepted = null;
			let acceptedPositions = null;
			for(let result of this.getAccepted(numPoints, node, matrix, points)){
				if(!result){
					yield false;
					checkpoint = performance.now();
				}else{
					[accepted, acceptedPositions] = result;
				}
			}

			let duration = performance.now() - checkpoint;
			if(duration > 4){
				yield false;
				checkpoint = performance.now();
			}

			points.data.position = acceptedPositions;

			let relevantAttributes = Object.keys(geometry.attributes).filter(a => !["position", "indices"].includes(a));
			for(const attributeName of relevantAttributes){
				const attribute = geometry.attributes[attributeName];
				const numElements = attribute.array.length / numPoints;

				if(numElements !== parseInt(numElements)){
					debugger;
				}

				const Type = attribute.array.constructor;

				let filteredBuffer = new Type(numElements * accepted.length);
				let source = attribute.array;

				for(let i = 0; i < accepted.length; i++){

					let index = accepted[i];

					let start = index * numElements;
					let end = start + numElements;
					let sub = source.subarray(start, end);

					filteredBuffer.set(sub, i * numElements);
				}

				points.data[attributeName] = filteredBuffer;
			}

			points.numPoints = accepted.length;

			target.points.add(points);
			target.size += points.numPoints;
		}

		yield true;
	};

	/**
	 * Calculate roughly whether a box and node intersect
	 * @param {BoxVolume} shape 
	 * @param {PointCloudOctreeGeometryNode} node 
	 * @returns {boolean} Intersects or not
	 */
	doesIntersect (shape, node) {
		const toShape = shape.matrixWorld.clone().invert();

		const bsWorld = (node.boundingBox.clone().applyMatrix4(this.pointcloud.matrixWorld))
			.getBoundingSphere(new THREE.Sphere());

		const start = new THREE.Vector3(-0.5, 0, 0);
		const end = new THREE.Vector3(0.5, 0, 0);

		const diff = new THREE.Line3(start, end)
			.applyMatrix4(shape.matrixWorld)
			.closestPointToPoint(bsWorld.center, true, new THREE.Vector3())
			.sub(bsWorld.center).applyQuaternion(shape.quaternion.conjugate());

		return (Math.abs(diff.y) < (bsWorld.radius + shape.scale.y/2))
			&& (Math.abs(diff.z) < (bsWorld.radius + shape.scale.z/2))
			&& (Math.abs(diff.x) < bsWorld.radius);
	}

	finishLevelThenCancel () {
		if (this.cancelRequested) {
			return;
		}

		this.maxDepth = this.highestLevelServed;
		this.cancelRequested = true;
	};

	cancel () {
		this.callback.onCancel();

		this.priorityQueue = new BinaryHeap(function (x) { return 1 / x.weight; });

		let index = this.pointcloud.pointPickerRequests.indexOf(this);
		if (index >= 0) {
			this.pointcloud.pointPickerRequests.splice(index, 1);
		}
	};
}
